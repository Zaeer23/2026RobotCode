package frc.dashboard;

import java.io.File;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Every song the dashboard knows about, and the playlists made from them.
 *
 * <p>Songs come from folders: the project's {@code music} folder, the user's Music folder, the top
 * level of Downloads, and any folder added from the dashboard. Single files can be added too.
 * Playlists are named lists of those files.
 *
 * <p>All of it is saved to {@code ~/.frc-dashboard/music.json}, outside the project, so playlists
 * survive a fresh clone and never end up committed. Songs whose files have gone are dropped
 * quietly when the library loads.
 *
 * <p>Methods are synchronized because the first scan runs off the Swing thread, so a big Music
 * folder does not freeze the window while it is read.
 */
final class MusicLibrary {

  /** A named list of songs, in the order they were added. */
  static final class Playlist {
    private String name;
    private final List<File> files = new ArrayList<>();

    private Playlist(String name) {
      this.name = name;
    }

    String name() {
      return name;
    }

    /** A copy, so callers can iterate while the library changes. */
    List<File> files() {
      synchronized (this) {
        return new ArrayList<>(files);
      }
    }

    int size() {
      synchronized (this) {
        return files.size();
      }
    }
  }

  /** Folders deeper than this are not searched for songs. */
  private static final int MAX_DEPTH = 5;

  /** Stops a huge folder (a whole drive, say) from making the library unusably slow. */
  private static final int MAX_SONGS = 10_000;

  private final File store;
  private final ObjectMapper json = new ObjectMapper();

  private final List<File> folders = new ArrayList<>();
  private final List<File> extraFiles = new ArrayList<>();
  private final List<Playlist> playlists = new ArrayList<>();
  private List<File> songs = List.of();
  private String problem;

  MusicLibrary(File store) {
    this.store = store;
  }

  /** The library saved in this user's home folder. */
  static MusicLibrary forThisUser() {
    return new MusicLibrary(
        new File(new File(System.getProperty("user.home"), ".frc-dashboard"), "music.json"));
  }

  // ---- Loading and saving -------------------------------------------------------------------------

  /** Reads the saved library, then scans every folder for songs. */
  synchronized void load() {
    folders.clear();
    extraFiles.clear();
    playlists.clear();
    problem = null;

    if (store.isFile()) {
      try {
        JsonNode root = json.readTree(store);
        for (JsonNode folder : root.path("folders")) {
          folders.add(new File(folder.asText()));
        }
        for (JsonNode file : root.path("files")) {
          extraFiles.add(new File(file.asText()));
        }
        for (JsonNode node : root.path("playlists")) {
          Playlist playlist = new Playlist(node.path("name").asText("Playlist"));
          for (JsonNode file : node.path("files")) {
            File song = new File(file.asText());
            if (song.isFile()) {
              playlist.files.add(song);
            }
          }
          playlists.add(playlist);
        }
      } catch (Exception e) {
        // A damaged file should not cost the user their songs; start fresh and say why.
        problem = "Couldn't read saved playlists (" + e.getMessage() + ")";
      }
    }
    rescan();
  }

  synchronized void save() {
    ObjectNode root = json.createObjectNode();
    root.put("version", 1);
    ArrayNode folderArray = root.putArray("folders");
    folders.forEach(folder -> folderArray.add(folder.getAbsolutePath()));
    ArrayNode fileArray = root.putArray("files");
    extraFiles.forEach(file -> fileArray.add(file.getAbsolutePath()));
    ArrayNode playlistArray = root.putArray("playlists");
    for (Playlist playlist : playlists) {
      ObjectNode node = playlistArray.addObject();
      node.put("name", playlist.name);
      ArrayNode files = node.putArray("files");
      playlist.files().forEach(file -> files.add(file.getAbsolutePath()));
    }
    try {
      store.getParentFile().mkdirs();
      json.writerWithDefaultPrettyPrinter().writeValue(store, root);
      problem = null;
    } catch (Exception e) {
      problem = "Couldn't save playlists (" + e.getMessage() + ")";
    }
  }

  /** Why the last load or save failed, or null. */
  synchronized String problem() {
    return problem;
  }

  /** Re-reads every folder. Picks up songs added to them since the dashboard opened. */
  synchronized void rescan() {
    Map<String, File> found = new LinkedHashMap<>();
    File home = new File(System.getProperty("user.home"));
    for (File musicFolder : projectMusicFolders()) {
      scan(musicFolder, MAX_DEPTH, found);
    }
    scan(new File(home, "Music"), MAX_DEPTH, found);
    scan(new File(home, "Downloads"), 0, found);
    for (File folder : folders) {
      scan(folder, MAX_DEPTH, found);
    }
    for (File file : extraFiles) {
      addIfPlayable(file, found);
    }
    // A playlist song in a folder nobody scans still belongs in All songs.
    for (Playlist playlist : playlists) {
      playlist.files().forEach(file -> addIfPlayable(file, found));
    }

    List<File> sorted = new ArrayList<>(found.values());
    sorted.sort(Comparator.comparing(file -> title(file).toLowerCase(Locale.ROOT)));
    songs = sorted;
  }

  /**
   * Every place the project's own {@code music} folder might be, relative to where this is running.
   *
   * <p>This used to be a bare {@code new File("music")}, which resolves against the process working
   * directory. That only matches the project root when the dashboard is launched from there, so the
   * three bundled songs vanished whenever it was started any other way — from the Driver Station,
   * from a shortcut, or from the staged bundle in build/dashboard. The tab looked broken when the
   * only thing wrong was where the process happened to be standing.
   *
   * <p>So: look next to the working directory, and also walk up from wherever this jar actually
   * lives. From {@code build/dashboard/dashboard.jar} the project root is two levels up.
   */
  private static List<File> projectMusicFolders() {
    List<File> roots = new ArrayList<>();
    roots.add(new File("music").getAbsoluteFile());

    File here = jarDirectory();
    for (int up = 0; here != null && up < 4; up++) {
      roots.add(new File(here, "music"));
      here = here.getParentFile();
    }

    List<File> existing = new ArrayList<>();
    for (File root : roots) {
      if (root.isDirectory()) {
        existing.add(root);
      }
    }
    return existing;
  }

  /** The folder holding this class's jar, or null when running from loose classes. */
  private static File jarDirectory() {
    try {
      java.net.URL source =
          MusicLibrary.class.getProtectionDomain().getCodeSource().getLocation();
      File path = new File(source.toURI());
      return path.isDirectory() ? path : path.getParentFile();
    } catch (Exception ignored) {
      // No code source, an opaque classloader, or a URI we cannot turn into a path. The working
      // directory entry above still stands; this is only ever an extra place to look.
      return null;
    }
  }

  private static void scan(File root, int depth, Map<String, File> found) {
    if (!root.isDirectory()) {
      return;
    }
    Deque<File[]> pending = new ArrayDeque<>();
    Deque<Integer> depths = new ArrayDeque<>();
    pending.push(new File[] {root});
    depths.push(depth);
    while (!pending.isEmpty() && found.size() < MAX_SONGS) {
      File[] batch = pending.pop();
      int remaining = depths.pop();
      for (File dir : batch) {
        File[] children = dir.listFiles();
        if (children == null) {
          continue;
        }
        List<File> subfolders = new ArrayList<>();
        for (File child : children) {
          if (child.isDirectory()) {
            if (remaining > 0 && !child.isHidden()) {
              subfolders.add(child);
            }
          } else {
            addIfPlayable(child, found);
          }
        }
        if (!subfolders.isEmpty()) {
          pending.push(subfolders.toArray(new File[0]));
          depths.push(remaining - 1);
        }
      }
    }
  }

  private static void addIfPlayable(File file, Map<String, File> found) {
    if (isPlayable(file)) {
      found.putIfAbsent(key(file), file.getAbsoluteFile());
    }
  }

  /** Same file however it was written down. Windows paths are case-insensitive. */
  private static String key(File file) {
    return file.getAbsoluteFile().toPath().normalize().toString().toLowerCase(Locale.ROOT);
  }

  // ---- Songs --------------------------------------------------------------------------------------

  /** Every known song, sorted by title. */
  synchronized List<File> songs() {
    return songs;
  }

  /** Adds single files. Returns the ones that are playable songs. */
  synchronized List<File> addFiles(Collection<File> files) {
    List<File> added = new ArrayList<>();
    for (File file : files) {
      if (isPlayable(file)) {
        File absolute = file.getAbsoluteFile();
        if (extraFiles.stream().noneMatch(f -> key(f).equals(key(absolute)))) {
          extraFiles.add(absolute);
        }
        added.add(absolute);
      }
    }
    rescan();
    save();
    return added;
  }

  /** Remembers a folder and scans it. Returns the songs found in it. */
  synchronized List<File> addFolder(File folder) {
    File absolute = folder.getAbsoluteFile();
    if (folders.stream().noneMatch(f -> key(f).equals(key(absolute)))) {
      folders.add(absolute);
    }
    Map<String, File> inFolder = new LinkedHashMap<>();
    scan(absolute, MAX_DEPTH, inFolder);
    rescan();
    save();
    List<File> found = new ArrayList<>(inFolder.values());
    found.sort(Comparator.comparing(file -> title(file).toLowerCase(Locale.ROOT)));
    return found;
  }

  // ---- Playlists ----------------------------------------------------------------------------------

  synchronized List<Playlist> playlists() {
    return new ArrayList<>(playlists);
  }

  /** A new empty playlist. The name is made unique if it is already taken. */
  synchronized Playlist create(String name) {
    Playlist playlist = new Playlist(uniqueName(name, null));
    playlists.add(playlist);
    save();
    return playlist;
  }

  synchronized void rename(Playlist playlist, String name) {
    playlist.name = uniqueName(name, playlist);
    save();
  }

  synchronized void delete(Playlist playlist) {
    playlists.remove(playlist);
    save();
  }

  /** Adds songs to the end of a playlist, skipping ones already in it. Returns how many went in. */
  synchronized int addTo(Playlist playlist, List<File> files) {
    int added = 0;
    synchronized (playlist) {
      for (File file : files) {
        boolean present = playlist.files.stream().anyMatch(f -> key(f).equals(key(file)));
        if (!present && isPlayable(file)) {
          playlist.files.add(file.getAbsoluteFile());
          added++;
        }
      }
    }
    save();
    return added;
  }

  synchronized void removeFrom(Playlist playlist, List<File> files) {
    synchronized (playlist) {
      playlist.files.removeIf(f -> files.stream().anyMatch(r -> key(r).equals(key(f))));
    }
    save();
  }

  private String uniqueName(String wanted, Playlist except) {
    String base = wanted == null || wanted.isBlank() ? "New playlist" : wanted.trim();
    String name = base;
    int n = 2;
    while (nameTaken(name, except)) {
      name = base + " " + n++;
    }
    return name;
  }

  private boolean nameTaken(String name, Playlist except) {
    return playlists.stream()
        .anyMatch(p -> p != except && p.name.equalsIgnoreCase(name));
  }

  // ---- Search -------------------------------------------------------------------------------------

  /**
   * The songs whose name contains every word of the query, in any order.
   *
   * <p>Matches on the file name and the folder it sits in, ignoring case and the dashes and
   * underscores file names use for spaces, so "daft punk" finds "Daft_Punk-One_More_Time.mp3"
   * and anything in a "Daft Punk" folder.
   */
  static List<File> search(List<File> songs, String query) {
    String[] words = normalise(query).split(" ");
    List<File> matches = new ArrayList<>();
    for (File song : songs) {
      File parent = song.getParentFile();
      String haystack = normalise(title(song) + " " + (parent == null ? "" : parent.getName()));
      boolean all = true;
      for (String word : words) {
        if (!word.isEmpty() && !haystack.contains(word)) {
          all = false;
          break;
        }
      }
      if (all) {
        matches.add(song);
      }
    }
    return matches;
  }

  private static String normalise(String text) {
    return text.toLowerCase(Locale.ROOT).replaceAll("[_\\-.]+", " ").replaceAll("\\s+", " ").trim();
  }

  /** The file name without its extension, which is what the dashboard shows as the title. */
  static String title(File file) {
    String name = file.getName();
    int dot = name.lastIndexOf('.');
    return dot > 0 ? name.substring(0, dot) : name;
  }

  /** MP3 is the only format the decoder handles. */
  static boolean isPlayable(File file) {
    return file != null && file.isFile() && file.getName().toLowerCase(Locale.ROOT).endsWith(".mp3");
  }
}
