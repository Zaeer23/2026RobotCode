package frc.dashboard;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.FloatControl;
import javax.sound.sampled.SourceDataLine;

import javazoom.jl.decoder.Bitstream;
import javazoom.jl.decoder.Decoder;
import javazoom.jl.decoder.Header;
import javazoom.jl.decoder.SampleBuffer;

/**
 * MP3 playback with volume, transport controls and a sample tap for the visualiser.
 *
 * <p>Decodes to raw PCM frame by frame rather than handing the file to a prebuilt player. That
 * costs a little code and buys two things a black-box player cannot give: a {@link SourceDataLine}
 * whose gain control is the volume slider, and the actual samples as they play, which is what the
 * visualiser draws. There is no other way to get both from plain Java — the JDK has no MP3
 * decoder, and JavaFX (which does) is not in the WPILib JDK.
 *
 * <p>Playback runs on its own daemon thread. Nothing here touches Swing directly; the UI passes in
 * callbacks and marshals to the EDT itself.
 *
 * <p>The player only knows its current queue. Which songs exist and which playlists they belong to
 * is {@link MusicLibrary}'s job; the Music tab hands the player a playlist's songs when one is
 * played.
 */
public class MusicPlayer {

  /** A track in the queue. Two tracks are the same if they are the same file. */
  public static class Track {
    public final File file;
    public final String title;

    Track(File file) {
      this.file = file;
      this.title = MusicLibrary.title(file);
    }

    @Override
    public boolean equals(Object other) {
      return other instanceof Track && ((Track) other).file.equals(file);
    }

    @Override
    public int hashCode() {
      return file.hashCode();
    }

    @Override
    public String toString() {
      return title;
    }
  }

  private final List<Track> playlist = new ArrayList<>();
  private final List<Integer> shuffleOrder = new ArrayList<>();

  private int shufflePosition = -1;
  /** Index in the queue of the current (or last) track. */
  private int position = -1;
  /** Index the user asked to play next, or -1. */
  private int requested = -1;
  /**
   * Bumped every time playback is started or stopped. A playback thread that sees a different
   * number than it started with has been replaced and quits, so a quick stop-then-play can never
   * leave two songs playing over each other.
   */
  private volatile int generation = 0;
  private volatile Track current;
  private volatile boolean playing = false;
  private volatile boolean paused = false;
  private volatile boolean skipRequested = false;
  private volatile boolean shuffle = true;
  private volatile float volume = 0.6f;

  private Thread playbackThread;
  private volatile SourceDataLine line;

  /**
   * Every sample decoded for the current track, mono, in a ring. Decoding runs ahead of the sound
   * card by however much the audio line buffers, so visualisers read from here at the line's
   * actual playback position rather than taking the newest samples, which would put the picture
   * a noticeable fraction of a second ahead of the music.
   */
  private final float[] ring = new float[1 << 16];
  private long ringWritten = 0L;
  private volatile long trackStartNanos = 0L;

  /** Set with {@link #requested} when the pick should not reshuffle the play order. */
  private boolean requestKeepsOrder = false;

  private Consumer<Track> onTrackChanged = t -> { };
  private Consumer<String> onError = e -> { };

  // ---- Queue ----------------------------------------------------------------

  /**
   * Replaces the queue with these songs. Whatever is playing keeps playing until it ends or
   * something else is picked; the next track comes from the new queue.
   */
  public synchronized void setQueue(List<File> files) {
    playlist.clear();
    for (File file : files) {
      if (MusicLibrary.isPlayable(file)) {
        playlist.add(new Track(file));
      }
    }
    reshuffle();
    position = -1;
    requested = -1;
  }

  /** A snapshot of the queue. */
  public synchronized List<Track> queue() {
    return new ArrayList<>(playlist);
  }

  public synchronized boolean isEmpty() {
    return playlist.isEmpty();
  }

  /** The track playing (or paused) now, or null. */
  public Track currentTrack() {
    return playing ? current : null;
  }

  /** Plays the queue's track at {@code index} straight away, interrupting whatever is playing. */
  public synchronized void playAt(int index) {
    if (index < 0 || index >= playlist.size()) {
      return;
    }
    requested = index;
    paused = false;
    if (playing) {
      skipRequested = true;
    } else {
      play();
    }
  }

  /** Rebuilds the random order. Called whenever the playlist changes. */
  private void reshuffle() {
    shuffleOrder.clear();
    for (int i = 0; i < playlist.size(); i++) {
      shuffleOrder.add(i);
    }
    Collections.shuffle(shuffleOrder);
    shufflePosition = -1;
  }

  // ---- Transport ------------------------------------------------------------

  public void setOnTrackChanged(Consumer<Track> listener) {
    this.onTrackChanged = listener == null ? t -> { } : listener;
  }

  public void setOnError(Consumer<String> listener) {
    this.onError = listener == null ? e -> { } : listener;
  }

  public synchronized void play() {
    if (playlist.isEmpty()) {
      onError.accept("Playlist is empty. Add some songs.");
      return;
    }
    if (paused) {
      paused = false;
      return;
    }
    if (playing) {
      return;
    }
    playing = true;
    skipRequested = false;
    int myGeneration = ++generation;
    playbackThread = new Thread(() -> playbackLoop(myGeneration), "music-player");
    playbackThread.setDaemon(true);
    playbackThread.start();
  }

  public void pause() {
    paused = true;
  }

  public void togglePause() {
    if (!playing) {
      play();
    } else {
      paused = !paused;
    }
  }

  public synchronized void stop() {
    playing = false;
    paused = false;
    skipRequested = true;
    generation++;
    current = null;
  }

  /**
   * Goes back a song, or to the start of this one if it has been playing for a few seconds, the
   * way every music player's back button works.
   */
  public synchronized void previous() {
    if (!playing || playlist.isEmpty()) {
      return;
    }
    boolean restart = System.nanoTime() - trackStartNanos > 3_000_000_000L;
    if (restart) {
      requested = Math.max(0, position);
      requestKeepsOrder = true;
    } else if (shuffle) {
      // nextTrack() steps forward one, so back two lands on the previous song.
      shufflePosition = Math.max(-1, shufflePosition - 2);
    } else {
      position = Math.floorMod(position - 2, playlist.size());
    }
    paused = false;
    skipRequested = true;
  }

  /** Skips to the next track. */
  public void next() {
    skipRequested = true;
    if (!playing) {
      play();
    }
  }

  public boolean isPlaying() {
    return playing && !paused;
  }

  public boolean isPaused() {
    return paused;
  }

  public void setShuffle(boolean on) {
    shuffle = on;
  }

  public boolean isShuffle() {
    return shuffle;
  }

  // ---- Volume ---------------------------------------------------------------

  /** @param value 0 to 1 */
  public void setVolume(float value) {
    volume = Math.max(0f, Math.min(1f, value));
    applyVolume();
  }

  public float getVolume() {
    return volume;
  }

  /**
   * Applies the volume to the open line.
   *
   * <p>MASTER_GAIN is in decibels, so a linear slider maps logarithmically. Without that
   * conversion the top 20% of the slider does almost everything and the rest feels dead.
   */
  private void applyVolume() {
    SourceDataLine current = line;
    if (current == null || !current.isOpen()) {
      return;
    }
    try {
      if (current.isControlSupported(FloatControl.Type.MASTER_GAIN)) {
        FloatControl gain = (FloatControl) current.getControl(FloatControl.Type.MASTER_GAIN);
        float db = volume <= 0.0001f
            ? gain.getMinimum()
            : (float) (20.0 * Math.log10(volume));
        gain.setValue(Math.max(gain.getMinimum(), Math.min(gain.getMaximum(), db)));
      }
    } catch (IllegalArgumentException ignored) {
      // Some mixers expose no gain control. Playback still works, just without volume.
    }
  }

  // ---- Visualiser feed ------------------------------------------------------

  /** The samples being heard right now, mono, -1..1. Empty when stopped. */
  public float[] latestSamples() {
    float[] window = new float[1152];
    return read(window) ? window : new float[0];
  }

  /**
   * Fills {@code window} with the samples that end at the sound card's current playback position.
   * Returns false, with silence, when nothing is playing or the music is paused.
   */
  public boolean read(float[] window) {
    SourceDataLine current = line;
    synchronized (ring) {
      if (!playing || paused || current == null || ringWritten == 0) {
        java.util.Arrays.fill(window, 0f);
        return false;
      }
      long heard = Math.min(current.getLongFramePosition(), ringWritten);
      long first = heard - window.length;
      long oldest = ringWritten - ring.length;
      for (int i = 0; i < window.length; i++) {
        long index = first + i;
        window[i] = index < 0 || index < oldest ? 0f : ring[(int) (index & (ring.length - 1))];
      }
      return true;
    }
  }


  // ---- Playback -------------------------------------------------------------

  private void playbackLoop(int myGeneration) {
    int failuresInARow = 0;
    while (playing && myGeneration == generation) {
      Track track = nextTrack();
      if (track == null) {
        break;
      }
      current = track;
      onTrackChanged.accept(track);
      try {
        playTrack(track, myGeneration);
        failuresInARow = 0;
      } catch (Exception e) {
        onError.accept("Could not play " + track.title + ": " + e.getMessage());
        // A queue where nothing decodes would otherwise spin through it forever.
        if (++failuresInARow >= Math.max(1, queue().size())) {
          break;
        }
      }
    }
    if (myGeneration == generation) {
      playing = false;
      current = null;
      }
  }

  private synchronized Track nextTrack() {
    if (playlist.isEmpty()) {
      return null;
    }

    // A track the user picked comes first; the shuffle carries on from there.
    if (requested >= 0 && requested < playlist.size()) {
      position = requested;
      requested = -1;
      if (requestKeepsOrder) {
        requestKeepsOrder = false;
        return playlist.get(position);
      }
      if (shuffle) {
        reshuffle();
        shuffleOrder.remove(Integer.valueOf(position));
        shuffleOrder.add(0, position);
        shufflePosition = 0;
      }
      return playlist.get(position);
    }
    requested = -1;

    if (!shuffle) {
      position = (position + 1) % playlist.size();
      return playlist.get(position);
    }

    shufflePosition++;
    if (shufflePosition >= shuffleOrder.size()) {
      // Reshuffle at the end of each pass so the order differs every time through.
      reshuffle();
      shufflePosition = 0;
    }
    if (shuffleOrder.isEmpty()) {
      return null;
    }
    position = Math.min(shuffleOrder.get(shufflePosition), playlist.size() - 1);
    return playlist.get(position);
  }

  /** Decodes one file frame by frame, writing PCM out and publishing samples as it goes. */
  private void playTrack(Track track, int myGeneration) throws Exception {
    skipRequested = false;
    boolean cutShort = false;
    int frames = 0;

    try (InputStream in = new BufferedInputStream(new FileInputStream(track.file), 64 * 1024)) {
      Bitstream bitstream = new Bitstream(in);
      Decoder decoder = new Decoder();
      line = null;

      try {
        Header header;
        while ((header = bitstream.readFrame()) != null) {
          if (!playing || skipRequested || myGeneration != generation) {
            cutShort = true;
            break;
          }
          while (paused && playing && !skipRequested && myGeneration == generation) {
            Thread.sleep(50);
          }

          SampleBuffer output = (SampleBuffer) decoder.decodeFrame(header, bitstream);
          short[] pcm = output.getBuffer();
          int sampleCount = output.getBufferLength();

          if (line == null) {
            synchronized (ring) {
              ringWritten = 0L;
            }
            trackStartNanos = System.nanoTime();
            line = openLine(output.getSampleFrequency(), output.getChannelCount());
            applyVolume();
          }

          publishSamples(pcm, sampleCount, output.getChannelCount());
          line.write(toBytes(pcm, sampleCount), 0, sampleCount * 2);
          bitstream.closeFrame();
          frames++;
        }
        // An empty or corrupt file "finishes" instantly. Treated as a failure, so a queue of
        // them stops instead of spinning through itself forever.
        if (frames == 0 && !cutShort) {
          throw new java.io.IOException("not a playable MP3");
        }
      } finally {
        if (line != null) {
          // A skipped or stopped song stops now; one that ended plays out its last buffer.
          if (cutShort) {
            line.flush();
          } else {
            line.drain();
          }
          line.close();
          line = null;
        }
        bitstream.close();
      }
    }
  }

  private SourceDataLine openLine(int sampleRate, int channels) throws Exception {
    AudioFormat format = new AudioFormat(sampleRate, 16, channels, true, false);
    SourceDataLine open =
        (SourceDataLine) AudioSystem.getLine(new DataLine.Info(SourceDataLine.class, format));
    open.open(format);
    open.start();
    return open;
  }

  /** Downmixes to mono and normalises, which is all the visualiser needs. */
  private void publishSamples(short[] pcm, int count, int channels) {
    int frames = channels > 1 ? count / channels : count;
    float[] mono = new float[frames];
    for (int i = 0; i < frames; i++) {
      if (channels > 1) {
        int left = pcm[i * channels];
        int right = pcm[i * channels + 1];
        mono[i] = ((left + right) / 2f) / 32768f;
      } else {
        mono[i] = pcm[i] / 32768f;
      }
    }
    synchronized (ring) {
      for (float sample : mono) {
        ring[(int) (ringWritten & (ring.length - 1))] = sample;
        ringWritten++;
      }
    }
  }

  private static byte[] toBytes(short[] pcm, int count) {
    byte[] bytes = new byte[count * 2];
    for (int i = 0; i < count; i++) {
      bytes[i * 2] = (byte) (pcm[i] & 0xff);
      bytes[i * 2 + 1] = (byte) ((pcm[i] >> 8) & 0xff);
    }
    return bytes;
  }
}
