package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.datatransfer.DataFlavor;
import java.awt.event.ActionEvent;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ThreadLocalRandom;

import javax.swing.AbstractAction;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.Icon;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.JToggleButton;
import javax.swing.KeyStroke;
import javax.swing.ListCellRenderer;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.TransferHandler;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.filechooser.FileNameExtensionFilter;

import com.formdev.flatlaf.FlatClientProperties;

/**
 * Music: every song on this laptop, searchable, sorted into playlists.
 *
 * <p>Laid out like any music app. Playlists down the left, with <em>All songs</em> at the top;
 * the selected one's songs in the middle with a search box over them; the player along the
 * bottom. Double-click a song to play its list from there. Select songs (shift and ctrl work)
 * and use <em>Add to playlist</em>, or right-click, to sort them into playlists.
 *
 * <p>Songs are MP3 files already on the laptop: the project's {@code music} folder, Music,
 * Downloads, and any file or folder added here or dragged onto the tab. Playlists are saved by
 * {@link MusicLibrary}.
 *
 * <p>Space plays and pauses and the arrow keys change song, anywhere on the tab except while
 * typing in the search box.
 */
public class MusicTab extends JPanel {

  /** The pseudo-playlist of every known song. */
  private static final Object ALL_SONGS = new Object() {
    @Override
    public String toString() {
      return "All songs";
    }
  };

  private final MusicPlayer player;
  private final MusicLibrary library;

  private final DefaultListModel<Object> sourceModel = new DefaultListModel<>();
  private final JList<Object> sourceList = new JList<>(sourceModel);
  private final DefaultListModel<File> trackModel = new DefaultListModel<>();
  private final TrackList trackList = new TrackList(trackModel);
  private final JTextField search = new JTextField();
  private final Card tracksCard = new Card("All songs", " ");

  private final JLabel selectionLabel = Ui.label(" ", Theme.ui(Font.PLAIN, 12f), Theme.TEXT_DIM);
  private final JLabel status = Ui.label(" ", Theme.ui(Font.PLAIN, 12f), Theme.GOOD);
  private final JButton addToButton = Ui.secondary("Add to playlist", Icons.plus(13, Theme.TEXT));
  private final JButton removeButton = Ui.secondary("Remove from playlist", Icons.minus(13, Theme.TEXT));

  private final JLabel trackTitle = Ui.label("Nothing playing", Theme.ui(Font.BOLD, 15f), Theme.TEXT);
  private final JLabel trackContext = Ui.label(" ", Theme.ui(Font.PLAIN, 12f), Theme.TEXT_DIM);
  private final JButton playPause = Ui.icon(Icons.play(18, Theme.TEXT), "Play / pause");

  private Object shown = ALL_SONGS;
  private List<File> shownSource = List.of();
  private String playingFrom = "";
  private boolean loading = true;
  private final Timer statusTimer = new Timer(5000, e -> status.setText(" "));

  public MusicTab(MusicPlayer player) {
    super(new BorderLayout(0, Theme.GAP));
    this.player = player;
    this.library = MusicLibrary.forThisUser();
    setBackground(Theme.BG);
    setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));

    JPanel top = new JPanel(new BorderLayout(Theme.GAP, 0));
    top.setOpaque(false);
    top.add(buildSidebar(), BorderLayout.WEST);
    top.add(buildTracks(), BorderLayout.CENTER);
    add(top, BorderLayout.CENTER);
    add(buildPlayerBar(), BorderLayout.SOUTH);

    statusTimer.setRepeats(false);
    installShortcuts();

    player.setOnTrackChanged(track -> SwingUtilities.invokeLater(() -> {
      trackTitle.setText(track.title);
      List<MusicPlayer.Track> queue = player.queue();
      int index = queue.indexOf(track);
      trackContext.setText(index >= 0
          ? (playingFrom.isEmpty() ? "" : "From " + playingFrom + "  ·  ")
              + "Song " + (index + 1) + " of " + queue.size()
          : " ");
      updatePlayPause();
      trackList.repaint();
    }));
    player.setOnError(message -> SwingUtilities.invokeLater(() -> say(message, true)));

    rebuildSources();
    refreshTracks();
    inBackground(library::load, () -> {
      loading = false;
      rebuildSources();
      refreshTracks();
      if (library.problem() != null) {
        say(library.problem(), true);
      }
    });
  }

  // ---- Layout -------------------------------------------------------------------------------------

  private JComponent buildSidebar() {
    JButton create = Ui.icon(Icons.plus(14, Theme.TEXT), "New playlist");
    create.addActionListener(e -> newPlaylist(List.of()));

    sourceList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
    sourceList.setBackground(Theme.SURFACE);
    sourceList.setCellRenderer(new SourceRenderer());
    sourceList.addListSelectionListener(e -> {
      Object selected = sourceList.getSelectedValue();
      if (!e.getValueIsAdjusting() && selected != null && selected != shown) {
        shown = selected;
        search.setText("");
        refreshTracks();
      }
    });
    sourceList.addMouseListener(new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent e) {
        int index = sourceList.locationToIndex(e.getPoint());
        if (index < 0 || !sourceList.getCellBounds(index, index).contains(e.getPoint())) {
          return;
        }
        if (SwingUtilities.isRightMouseButton(e)) {
          sourceList.setSelectedIndex(index);
          sourceMenu(sourceModel.get(index)).show(sourceList, e.getX(), e.getY());
        } else if (e.getClickCount() == 2) {
          playView(-1);
        }
      }
    });

    Card card = new Card("Playlists", "Right-click to rename or delete");
    card.addHeaderRight(create);
    card.content(scroll(sourceList));
    card.setPreferredSize(new Dimension(270, 10));
    return card;
  }

  private JComponent buildTracks() {
    search.putClientProperty(FlatClientProperties.PLACEHOLDER_TEXT, "Search songs…  (Ctrl+F)");
    search.putClientProperty(FlatClientProperties.TEXT_FIELD_LEADING_ICON,
        Icons.search(14, Theme.TEXT_DIM));
    search.putClientProperty(FlatClientProperties.TEXT_FIELD_SHOW_CLEAR_BUTTON, true);
    search.setPreferredSize(new Dimension(260, 32));
    search.getDocument().addDocumentListener(new DocumentListener() {
      @Override
      public void insertUpdate(DocumentEvent e) {
        refreshTracks();
      }

      @Override
      public void removeUpdate(DocumentEvent e) {
        refreshTracks();
      }

      @Override
      public void changedUpdate(DocumentEvent e) {
        refreshTracks();
      }
    });
    search.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0), "clearSearch");
    search.getActionMap().put("clearSearch", action(() -> search.setText("")));
    // Enter in the search box plays the top result.
    search.addActionListener(e -> {
      if (!trackModel.isEmpty()) {
        playView(0);
      }
    });

    JButton addSongs = Ui.secondary("Add songs", Icons.plus(13, Theme.TEXT));
    addSongs.setToolTipText("Add MP3 files from this laptop");
    addSongs.addActionListener(e -> chooseFiles());
    JButton addFolder = Ui.secondary("Add folder", Icons.folder(14, Theme.TEXT));
    addFolder.setToolTipText("Add every MP3 in a folder, and keep watching it");
    addFolder.addActionListener(e -> chooseFolder());
    JButton rescan = Ui.icon(Icons.restart(14, Theme.TEXT), "Look for new songs in your folders");
    rescan.addActionListener(e -> inBackground(library::rescan, () -> {
      refreshTracks();
      say("Found " + library.songs().size() + " songs", false);
    }));

    tracksCard.addHeaderRight(search);
    tracksCard.addHeaderRight(addSongs);
    tracksCard.addHeaderRight(addFolder);
    tracksCard.addHeaderRight(rescan);

    trackList.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
    trackList.setBackground(Theme.SURFACE);
    trackList.setCellRenderer(new TrackRenderer());
    trackList.setFixedCellWidth(10);
    trackList.addListSelectionListener(e -> updateActions());
    trackList.addMouseListener(new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent e) {
        int index = trackList.locationToIndex(e.getPoint());
        boolean onRow = index >= 0 && trackList.getCellBounds(index, index).contains(e.getPoint());
        if (SwingUtilities.isRightMouseButton(e) && onRow) {
          if (!trackList.isSelectedIndex(index)) {
            trackList.setSelectedIndex(index);
          }
          trackMenu().show(trackList, e.getX(), e.getY());
        } else if (e.getClickCount() == 2 && onRow && SwingUtilities.isLeftMouseButton(e)) {
          playView(index);
        }
      }
    });
    trackList.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, 0), "play");
    trackList.getActionMap().put("play", action(() -> playView(trackList.getSelectedIndex())));
    trackList.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE, 0), "remove");
    trackList.getActionMap().put("remove", action(this::removeSelected));

    addToButton.addActionListener(e -> addToMenu(selectedSongs())
        .show(addToButton, 0, addToButton.getHeight()));
    removeButton.addActionListener(e -> removeSelected());

    JPanel actions = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 0));
    actions.setOpaque(false);
    actions.add(selectionLabel);
    actions.add(addToButton);
    actions.add(removeButton);

    JPanel footer = new JPanel(new BorderLayout(12, 0));
    footer.setOpaque(false);
    footer.setBorder(BorderFactory.createCompoundBorder(
        BorderFactory.createMatteBorder(1, 0, 0, 0, Theme.BORDER),
        BorderFactory.createEmptyBorder(10, 0, 0, 0)));
    footer.add(actions, BorderLayout.WEST);
    footer.add(status, BorderLayout.EAST);

    tracksCard.content(scroll(trackList));
    tracksCard.add(footer, BorderLayout.SOUTH);
    return tracksCard;
  }

  private JComponent buildPlayerBar() {
    playPause.addActionListener(e -> togglePlay());
    JButton previous = Ui.icon(Icons.previous(16, Theme.TEXT), "Previous song (\u2190)");
    previous.addActionListener(e -> player.previous());
    JButton next = Ui.icon(Icons.next(16, Theme.TEXT), "Next song (\u2192)");
    next.addActionListener(e -> player.next());
    JButton stop = Ui.icon(Icons.stop(16, Theme.TEXT), "Stop");
    stop.addActionListener(e -> {
      player.stop();
      trackTitle.setText("Nothing playing");
      trackContext.setText(" ");
      updatePlayPause();
      trackList.repaint();
    });

    JToggleButton shuffle = new JToggleButton("Shuffle", Icons.shuffle(14, Theme.TEXT), true);
    shuffle.putClientProperty(FlatClientProperties.STYLE,
        "arc: 999; focusWidth: 0; innerFocusWidth: 0; margin: 6,12,6,12;"
            + "selectedBackground: #1f4e6b; selectedForeground: #e7eaef");
    shuffle.addActionListener(e -> player.setShuffle(shuffle.isSelected()));

    JSlider volume = new JSlider(0, 100, (int) (player.getVolume() * 100));
    volume.setOpaque(false);
    volume.setPreferredSize(new Dimension(130, 28));
    volume.addChangeListener(e -> player.setVolume(volume.getValue() / 100f));

    JPanel controls = new JPanel(new FlowLayout(FlowLayout.RIGHT, 8, 0));
    controls.setOpaque(false);
    controls.add(previous);
    controls.add(playPause);
    controls.add(next);
    controls.add(stop);
    controls.add(Box.createHorizontalStrut(6));
    controls.add(shuffle);
    controls.add(Box.createHorizontalStrut(6));
    controls.add(new JLabel(Icons.volume(16, Theme.TEXT_DIM)));
    controls.add(volume);
    JPanel controlsCentred = new JPanel(new java.awt.GridBagLayout());
    controlsCentred.setOpaque(false);
    controlsCentred.add(controls);

    JPanel titles = new JPanel();
    titles.setLayout(new BoxLayout(titles, BoxLayout.Y_AXIS));
    titles.setOpaque(false);
    titles.add(Ui.caption("Now playing"));
    titles.add(Box.createVerticalStrut(3));
    titles.add(trackTitle);
    titles.add(Box.createVerticalStrut(2));
    titles.add(trackContext);
    // Vertically centred, but hugging the cover art rather than floating in the middle.
    JPanel titlesCentred = new JPanel(new java.awt.GridBagLayout());
    titlesCentred.setOpaque(false);
    java.awt.GridBagConstraints west = new java.awt.GridBagConstraints();
    west.anchor = java.awt.GridBagConstraints.WEST;
    west.weightx = 1.0;
    titlesCentred.add(titles, west);

    JPanel left = new JPanel(new BorderLayout(14, 0));
    left.setOpaque(false);
    left.add(new CoverArt(64), BorderLayout.WEST);
    left.add(titlesCentred, BorderLayout.CENTER);
    left.setPreferredSize(new Dimension(360, 64));

    Visualizer visualizer = new Visualizer(player);
    visualizer.setPreferredSize(new Dimension(200, 64));

    JPanel bar = new JPanel(new BorderLayout(20, 0));
    bar.setOpaque(false);
    bar.add(left, BorderLayout.WEST);
    bar.add(visualizer, BorderLayout.CENTER);
    bar.add(controlsCentred, BorderLayout.EAST);

    Card card = new Card();
    card.setPadding(14, 16, 14, 18);
    card.content(bar);
    return card;
  }

  private void installShortcuts() {
    getInputMap(WHEN_ANCESTOR_OF_FOCUSED_COMPONENT)
        .put(KeyStroke.getKeyStroke(KeyEvent.VK_F, InputEvent.CTRL_DOWN_MASK), "focusSearch");
    getActionMap().put("focusSearch", action(() -> {
      search.requestFocusInWindow();
      search.selectAll();
    }));

    // Space and the arrows work wherever focus is on this tab, except in the search box where
    // they type. The lists get the bindings directly because they would otherwise take them.
    for (JComponent target : new JComponent[] {this, trackList, sourceList}) {
      int condition = target == this ? WHEN_ANCESTOR_OF_FOCUSED_COMPONENT : WHEN_FOCUSED;
      target.getInputMap(condition).put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "playPause");
      target.getInputMap(condition).put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, 0), "previous");
      target.getInputMap(condition).put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, 0), "next");
      target.getActionMap().put("playPause", action(this::togglePlay));
      target.getActionMap().put("previous", action(player::previous));
      target.getActionMap().put("next", action(player::next));
    }

    DropHandler drop = new DropHandler();
    setTransferHandler(drop);
    trackList.setTransferHandler(drop);
    sourceList.setTransferHandler(drop);
  }

  private static JScrollPane scroll(JComponent view) {
    JScrollPane scroll = new JScrollPane(view);
    scroll.setBorder(null);
    scroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    scroll.getVerticalScrollBar().setUnitIncrement(16);
    return scroll;
  }

  private static AbstractAction action(Runnable run) {
    return new AbstractAction() {
      @Override
      public void actionPerformed(ActionEvent e) {
        run.run();
      }
    };
  }

  // ---- Showing ------------------------------------------------------------------------------------

  private void rebuildSources() {
    Object keep = shown;
    sourceModel.clear();
    sourceModel.addElement(ALL_SONGS);
    for (MusicLibrary.Playlist playlist : library.playlists()) {
      sourceModel.addElement(playlist);
    }
    if (!sourceModel.contains(keep)) {
      keep = ALL_SONGS;
    }
    shown = keep;
    sourceList.setSelectedValue(keep, true);
  }

  private String nameOf(Object view) {
    return view instanceof MusicLibrary.Playlist ? ((MusicLibrary.Playlist) view).name() : "All songs";
  }

  private List<File> songsOf(Object view) {
    return view instanceof MusicLibrary.Playlist
        ? ((MusicLibrary.Playlist) view).files()
        : library.songs();
  }

  /** Re-reads the shown list and applies the search. */
  private void refreshTracks() {
    shownSource = songsOf(shown);
    String query = search.getText().trim();
    List<File> visible = query.isEmpty() ? shownSource : MusicLibrary.search(shownSource, query);

    List<File> keepSelected = selectedSongs();
    trackModel.clear();
    trackModel.addAll(visible);
    for (File file : keepSelected) {
      int index = visible.indexOf(file);
      if (index >= 0) {
        trackList.addSelectionInterval(index, index);
      }
    }

    tracksCard.setTitle(nameOf(shown));
    int total = shownSource.size();
    String count = total == 1 ? "1 song" : total + " songs";
    tracksCard.setSubtitle(loading ? "Looking for songs…"
        : query.isEmpty() ? count : visible.size() + " of " + count + " match “" + query + "”");

    if (loading) {
      trackList.emptyText = "Looking for songs on this laptop…";
    } else if (!query.isEmpty()) {
      trackList.emptyText = "No songs match “" + query + "”.";
    } else if (shown == ALL_SONGS) {
      trackList.emptyText = "No songs yet. Drag MP3s or folders here, or use Add songs or Add "
          + "folder.";
    } else {
      trackList.emptyText = "This playlist is empty. Find songs in All songs, select them, and use "
          + "Add to playlist.";
    }
    trackList.repaint();
    sourceList.repaint();
    updateActions();
  }

  private List<File> selectedSongs() {
    return new ArrayList<>(trackList.getSelectedValuesList());
  }

  private void updateActions() {
    int selected = trackList.getSelectedIndices().length;
    boolean inPlaylist = shown instanceof MusicLibrary.Playlist;
    String hint = inPlaylist
        ? "Select songs to remove them or copy them to another playlist"
        : "Select songs to sort them into playlists";
    selectionLabel.setText(selected == 0 ? hint
        : selected == 1 ? "1 selected" : selected + " selected");
    addToButton.setVisible(selected > 0);
    removeButton.setVisible(selected > 0 && inPlaylist);
  }

  /** Play or pause; with nothing playing yet, starts the list on screen. */
  private void togglePlay() {
    if (player.currentTrack() == null) {
      playView(-1);
    } else {
      player.togglePause();
    }
    updatePlayPause();
  }

  private void updatePlayPause() {
    boolean on = player.isPlaying();
    playPause.setIcon(on ? Icons.pause(18, Theme.TEXT) : Icons.play(18, Theme.TEXT));
  }

  /** A short confirmation (or problem) in the corner of the song list. */
  private void say(String message, boolean problem) {
    status.setForeground(problem ? Theme.WARN : Theme.GOOD);
    status.setText(message);
    statusTimer.restart();
  }

  // ---- Playing ------------------------------------------------------------------------------------

  /**
   * Plays the shown list, starting at a row of the (possibly filtered) song list.
   *
   * <p>The whole list is queued, not just the search results, so after the song you searched for
   * the rest of the playlist carries on. With no row (-1) it starts at the top, or anywhere when
   * shuffling.
   */
  private void playView(int row) {
    if (shownSource.isEmpty()) {
      say(shown == ALL_SONGS ? "No songs to play yet" : "This playlist is empty", true);
      return;
    }
    int start;
    if (row >= 0 && row < trackModel.size()) {
      start = Math.max(0, shownSource.indexOf(trackModel.get(row)));
    } else {
      start = player.isShuffle() ? ThreadLocalRandom.current().nextInt(shownSource.size()) : 0;
    }
    playingFrom = nameOf(shown);
    player.setQueue(shownSource);
    player.playAt(start);
    updatePlayPause();
  }

  // ---- Editing ------------------------------------------------------------------------------------

  private JPopupMenu trackMenu() {
    List<File> songs = selectedSongs();
    JPopupMenu menu = new JPopupMenu();
    JMenuItem play = new JMenuItem("Play", Icons.play(12, Theme.TEXT));
    play.addActionListener(e -> playView(trackList.getSelectedIndex()));
    menu.add(play);

    JMenu addTo = new JMenu("Add to playlist");
    addTo.setIcon(Icons.plus(12, Theme.TEXT));
    fillAddTo(addTo, songs);
    menu.add(addTo);

    if (shown instanceof MusicLibrary.Playlist) {
      JMenuItem remove = new JMenuItem("Remove from this playlist", Icons.minus(12, Theme.TEXT));
      remove.addActionListener(e -> removeSelected());
      menu.add(remove);
    }
    return menu;
  }

  private JPopupMenu addToMenu(List<File> songs) {
    JPopupMenu menu = new JPopupMenu();
    JMenu holder = new JMenu();
    fillAddTo(holder, songs);
    for (Component item : holder.getMenuComponents()) {
      menu.add(item);
    }
    return menu;
  }

  private void fillAddTo(JMenu menu, List<File> songs) {
    for (MusicLibrary.Playlist playlist : library.playlists()) {
      if (playlist == shown) {
        continue;
      }
      JMenuItem item = new JMenuItem(playlist.name(), Icons.playlist(12, Theme.TEXT_DIM));
      item.addActionListener(e -> addTo(playlist, songs));
      menu.add(item);
    }
    if (menu.getMenuComponentCount() > 0) {
      menu.addSeparator();
    }
    JMenuItem fresh = new JMenuItem("New playlist…", Icons.plus(12, Theme.ACCENT));
    fresh.addActionListener(e -> newPlaylist(songs));
    menu.add(fresh);
  }

  private void addTo(MusicLibrary.Playlist playlist, List<File> songs) {
    int added = library.addTo(playlist, songs);
    int skipped = songs.size() - added;
    say(added == 0 ? "Already in " + playlist.name()
        : "Added " + plural(added) + " to " + playlist.name()
            + (skipped > 0 ? " (" + skipped + " already there)" : ""), added == 0);
    sourceList.repaint();
  }

  private void removeSelected() {
    if (!(shown instanceof MusicLibrary.Playlist)) {
      return;
    }
    List<File> songs = selectedSongs();
    if (songs.isEmpty()) {
      return;
    }
    MusicLibrary.Playlist playlist = (MusicLibrary.Playlist) shown;
    library.removeFrom(playlist, songs);
    trackList.clearSelection();
    refreshTracks();
    say("Removed " + plural(songs.size()) + " from " + playlist.name(), false);
  }

  /** Asks for a name, creates the playlist, and puts {@code songs} in it (if any). */
  private void newPlaylist(List<File> songs) {
    String name = (String) JOptionPane.showInputDialog(this, "Name the new playlist:",
        "New playlist", JOptionPane.PLAIN_MESSAGE, null, null, "");
    if (name == null) {
      return;
    }
    MusicLibrary.Playlist playlist = library.create(name);
    rebuildSources();
    if (songs.isEmpty()) {
      sourceList.setSelectedValue(playlist, true);
      say("Created " + playlist.name() + ". Add songs from All songs.", false);
    } else {
      library.addTo(playlist, songs);
      sourceList.repaint();
      say("Created " + playlist.name() + " with " + plural(songs.size()), false);
    }
  }

  private JPopupMenu sourceMenu(Object view) {
    JPopupMenu menu = new JPopupMenu();
    JMenuItem play = new JMenuItem("Play", Icons.play(12, Theme.TEXT));
    play.addActionListener(e -> playView(-1));
    menu.add(play);
    if (view instanceof MusicLibrary.Playlist) {
      MusicLibrary.Playlist playlist = (MusicLibrary.Playlist) view;
      JMenuItem rename = new JMenuItem("Rename…");
      rename.addActionListener(e -> {
        Object name = JOptionPane.showInputDialog(this, "New name:", "Rename playlist",
            JOptionPane.PLAIN_MESSAGE, null, null, playlist.name());
        if (name != null && !name.toString().isBlank()) {
          library.rename(playlist, name.toString());
          refreshTracks();
        }
      });
      menu.add(rename);
      JMenuItem delete = new JMenuItem("Delete", Icons.cross(12, Theme.BAD));
      delete.addActionListener(e -> {
        int answer = JOptionPane.showConfirmDialog(this,
            "Delete the playlist “" + playlist.name() + "”?\n"
                + "The songs stay on the laptop; only the list is removed.",
            "Delete playlist", JOptionPane.OK_CANCEL_OPTION, JOptionPane.WARNING_MESSAGE);
        if (answer == JOptionPane.OK_OPTION) {
          library.delete(playlist);
          rebuildSources();
          refreshTracks();
          say("Deleted " + playlist.name(), false);
        }
      });
      menu.add(delete);
    }
    return menu;
  }

  private void chooseFiles() {
    JFileChooser chooser = new JFileChooser(new File(System.getProperty("user.home"), "Music"));
    chooser.setMultiSelectionEnabled(true);
    chooser.setFileFilter(new FileNameExtensionFilter("MP3 audio", "mp3"));
    if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) {
      return;
    }
    List<File> picked = List.of(chooser.getSelectedFiles());
    Object target = shown;
    List<List<File>> result = new ArrayList<>();
    inBackground(() -> result.add(library.addFiles(picked)), () -> afterAdding(result.get(0), target));
  }

  private void chooseFolder() {
    JFileChooser chooser = new JFileChooser(new File(System.getProperty("user.home")));
    chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
    chooser.setDialogTitle("Add a folder of songs");
    if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) {
      return;
    }
    File folder = chooser.getSelectedFile();
    Object target = shown;
    List<List<File>> result = new ArrayList<>();
    inBackground(() -> result.add(library.addFolder(folder)), () -> afterAdding(result.get(0), target));
  }

  /** MP3s and folders dragged onto the tab, from File Explorer for instance. */
  private final class DropHandler extends TransferHandler {
    @Override
    public boolean canImport(TransferSupport support) {
      return support.isDataFlavorSupported(DataFlavor.javaFileListFlavor);
    }

    @Override
    public boolean importData(TransferSupport support) {
      if (!canImport(support)) {
        return false;
      }
      try {
        @SuppressWarnings("unchecked")
        List<File> dropped =
            (List<File>) support.getTransferable().getTransferData(DataFlavor.javaFileListFlavor);
        addDropped(dropped);
        return true;
      } catch (Exception e) {
        say("Couldn't add those files", true);
        return false;
      }
    }
  }

  private void addDropped(List<File> dropped) {
    List<File> files = new ArrayList<>();
    List<File> folders = new ArrayList<>();
    for (File file : dropped) {
      (file.isDirectory() ? folders : files).add(file);
    }
    Object target = shown;
    List<File> added = new ArrayList<>();
    inBackground(() -> {
      if (!files.isEmpty()) {
        added.addAll(library.addFiles(files));
      }
      for (File folder : folders) {
        added.addAll(library.addFolder(folder));
      }
    }, () -> afterAdding(added, target));
  }

  /** Files added while a playlist is open go into that playlist too. */
  private void afterAdding(List<File> added, Object target) {
    if (added.isEmpty()) {
      say("No MP3 files found there", true);
      return;
    }
    if (target instanceof MusicLibrary.Playlist) {
      library.addTo((MusicLibrary.Playlist) target, added);
      say("Added " + plural(added.size()) + " to " + ((MusicLibrary.Playlist) target).name(), false);
    } else {
      say("Added " + plural(added.size()), false);
    }
    refreshTracks();
  }

  private static String plural(int songs) {
    return songs == 1 ? "1 song" : songs + " songs";
  }

  /** Runs slow library work (disk scans) off the Swing thread, then updates the UI on it. */
  private static void inBackground(Runnable work, Runnable then) {
    Thread thread = new Thread(() -> {
      try {
        work.run();
      } finally {
        SwingUtilities.invokeLater(then);
      }
    }, "music-library");
    thread.setDaemon(true);
    thread.start();
  }

  // ---- Rendering ----------------------------------------------------------------------------------

  /** A list that explains itself when it is empty, rather than showing a blank box. */
  private static final class TrackList extends JList<File> {
    String emptyText = "";

    TrackList(DefaultListModel<File> model) {
      super(model);
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      if (getModel().getSize() == 0 && !emptyText.isEmpty()) {
        Graphics2D g2 = (Graphics2D) g.create();
        Theme.smooth(g2);
        g2.setFont(Theme.ui(Font.PLAIN, 13f));
        g2.setColor(Theme.TEXT_DIM);
        FontMetrics fm = g2.getFontMetrics();
        int y = 40;
        for (String line : wrap(fm, emptyText, Math.min(460, getWidth() - 40))) {
          g2.drawString(line, (getWidth() - fm.stringWidth(line)) / 2, y);
          y += fm.getHeight();
        }
        g2.dispose();
      }
    }

    private static List<String> wrap(FontMetrics fm, String text, int width) {
      List<String> lines = new ArrayList<>();
      StringBuilder line = new StringBuilder();
      for (String word : text.split(" ")) {
        if (line.length() > 0 && fm.stringWidth(line + " " + word) > width) {
          lines.add(line.toString());
          line = new StringBuilder();
        }
        line.append(line.length() == 0 ? "" : " ").append(word);
      }
      lines.add(line.toString());
      return lines;
    }
  }

  /** Song row: number (or a play mark if it is the one playing), title, and its folder. */
  private final class TrackRenderer extends JPanel implements ListCellRenderer<File> {
    private final Icon playing = Icons.play(11, Theme.ACCENT);
    private final Icon blank = Icons.blank(11);
    private final JLabel number = Ui.label("", Theme.numeric(Font.PLAIN, 12f), Theme.TEXT_FAINT);
    private final JLabel title = new JLabel();
    private final JLabel folder = Ui.label("", Theme.ui(Font.PLAIN, 12f), Theme.TEXT_FAINT);

    TrackRenderer() {
      super(new BorderLayout(12, 0));
      setBorder(BorderFactory.createEmptyBorder(8, 12, 8, 14));
      number.setPreferredSize(new Dimension(52, 18));
      number.setIconTextGap(10);
      add(number, BorderLayout.WEST);
      add(title, BorderLayout.CENTER);
      add(folder, BorderLayout.EAST);
    }

    @Override
    public Component getListCellRendererComponent(JList<? extends File> list, File song,
        int index, boolean isSelected, boolean cellHasFocus) {
      MusicPlayer.Track now = player.currentTrack();
      boolean current = now != null && now.file.equals(song);
      number.setText(String.format(Locale.ROOT, "%02d", index + 1));
      number.setIcon(current ? playing : blank);
      title.setText(MusicLibrary.title(song));
      title.setFont(Theme.ui(current ? Font.BOLD : Font.PLAIN, 13f));
      title.setForeground(current ? Theme.ACCENT : Theme.TEXT);
      File parent = song.getParentFile();
      folder.setText(parent == null ? "" : parent.getName());
      setBackground(isSelected ? list.getSelectionBackground() : Theme.SURFACE);
      return this;
    }
  }

  /** Playlist row: icon, name, and how many songs. */
  private final class SourceRenderer extends JPanel implements ListCellRenderer<Object> {
    private final JLabel name = new JLabel();
    private final JLabel count = Ui.label("", Theme.numeric(Font.PLAIN, 12f), Theme.TEXT_FAINT);

    SourceRenderer() {
      super(new BorderLayout(10, 0));
      setBorder(BorderFactory.createEmptyBorder(9, 12, 9, 12));
      name.setIconTextGap(10);
      add(name, BorderLayout.CENTER);
      add(count, BorderLayout.EAST);
    }

    @Override
    public Component getListCellRendererComponent(JList<?> list, Object value, int index,
        boolean isSelected, boolean cellHasFocus) {
      boolean all = value == ALL_SONGS;
      name.setText(nameOf(value));
      name.setIcon(all ? Icons.music(15, isSelected ? Theme.ACCENT : Theme.TEXT_DIM)
          : Icons.playlist(15, isSelected ? Theme.ACCENT : Theme.TEXT_DIM));
      name.setFont(Theme.ui(all || isSelected ? Font.BOLD : Font.PLAIN, 13f));
      name.setForeground(isSelected ? Theme.TEXT : Theme.TEXT);
      int songs = all ? library.songs().size() : ((MusicLibrary.Playlist) value).size();
      count.setText(loading && all ? "" : String.valueOf(songs));
      setBackground(isSelected ? list.getSelectionBackground() : Theme.SURFACE);
      return this;
    }
  }

  /** A gradient tile with a note on it, where album art would be. */
  private static final class CoverArt extends JComponent {
    private final int size;

    CoverArt(int size) {
      this.size = size;
      setPreferredSize(new Dimension(size, size));
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      int y = (getHeight() - size) / 2;
      g2.setPaint(new java.awt.GradientPaint(0, y, Theme.ACCENT, size, y + size, Theme.ROUTE_INTAKE));
      g2.fill(new java.awt.geom.RoundRectangle2D.Double(0, y, size, size, 14, 14));
      int icon = size / 2;
      Icons.music(icon, Theme.alpha(Theme.TEXT_ON_ACCENT, 200))
          .paintIcon(this, g2, (size - icon) / 2, y + (size - icon) / 2);
      g2.dispose();
    }
  }
}
