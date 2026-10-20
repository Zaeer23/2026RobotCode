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
 */
public class MusicPlayer {

  /** A track in the playlist. */
  public static class Track {
    public final File file;
    public final String title;

    Track(File file) {
      this.file = file;
      String name = file.getName();
      int dot = name.lastIndexOf('.');
      this.title = dot > 0 ? name.substring(0, dot) : name;
    }

    @Override
    public String toString() {
      return title;
    }
  }

  private final List<Track> playlist = new ArrayList<>();
  private final List<Integer> shuffleOrder = new ArrayList<>();

  private int shufflePosition = -1;
  private volatile boolean playing = false;
  private volatile boolean paused = false;
  private volatile boolean skipRequested = false;
  private volatile boolean shuffle = true;
  private volatile float volume = 0.6f;

  private Thread playbackThread;
  private SourceDataLine line;

  /** Latest decoded samples, for the visualiser. Mono, normalised to -1..1. */
  private volatile float[] latestSamples = new float[0];

  private Consumer<Track> onTrackChanged = t -> { };
  private Consumer<String> onError = e -> { };

  // ---- Playlist -------------------------------------------------------------

  /** Adds a file if it looks like audio we can decode. Returns false if not. */
  public boolean addTrack(File file) {
    if (file == null || !file.isFile()) {
      return false;
    }
    String lower = file.getName().toLowerCase();
    if (!lower.endsWith(".mp3")) {
      return false;
    }
    playlist.add(new Track(file));
    reshuffle();
    return true;
  }

  public void removeTrack(int index) {
    if (index >= 0 && index < playlist.size()) {
      playlist.remove(index);
      reshuffle();
    }
  }

  public List<Track> playlist() {
    return Collections.unmodifiableList(playlist);
  }

  public boolean isEmpty() {
    return playlist.isEmpty();
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
    playbackThread = new Thread(this::playbackLoop, "music-player");
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
    latestSamples = new float[0];
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

  /** Most recent decoded samples, mono, -1..1. Empty when stopped. */
  public float[] latestSamples() {
    return latestSamples;
  }

  // ---- Playback -------------------------------------------------------------

  private void playbackLoop() {
    while (playing) {
      Track track = nextTrack();
      if (track == null) {
        break;
      }
      onTrackChanged.accept(track);
      try {
        playTrack(track);
      } catch (Exception e) {
        onError.accept("Could not play " + track.title + ": " + e.getMessage());
      }
    }
    playing = false;
    latestSamples = new float[0];
  }

  private Track nextTrack() {
    if (playlist.isEmpty()) {
      return null;
    }
    if (!shuffle) {
      shufflePosition = (shufflePosition + 1) % playlist.size();
      return playlist.get(shufflePosition);
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
    int index = shuffleOrder.get(Math.min(shufflePosition, shuffleOrder.size() - 1));
    return playlist.get(Math.min(index, playlist.size() - 1));
  }

  /** Decodes one file frame by frame, writing PCM out and publishing samples as it goes. */
  private void playTrack(Track track) throws Exception {
    skipRequested = false;

    try (InputStream in = new BufferedInputStream(new FileInputStream(track.file), 64 * 1024)) {
      Bitstream bitstream = new Bitstream(in);
      Decoder decoder = new Decoder();
      line = null;

      try {
        Header header;
        while ((header = bitstream.readFrame()) != null) {
          if (!playing || skipRequested) {
            break;
          }
          while (paused && playing && !skipRequested) {
            Thread.sleep(50);
          }

          SampleBuffer output = (SampleBuffer) decoder.decodeFrame(header, bitstream);
          short[] pcm = output.getBuffer();
          int sampleCount = output.getBufferLength();

          if (line == null) {
            line = openLine(output.getSampleFrequency(), output.getChannelCount());
            applyVolume();
          }

          publishSamples(pcm, sampleCount, output.getChannelCount());
          line.write(toBytes(pcm, sampleCount), 0, sampleCount * 2);
          bitstream.closeFrame();
        }
      } finally {
        if (line != null) {
          line.drain();
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
    latestSamples = mono;
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
