package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.io.File;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JSplitPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.UIManager;
import javax.swing.filechooser.FileNameExtensionFilter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Driver station dashboard.
 *
 * <p>Runs on the laptop, not the robot — the roboRIO is headless. It is a NetworkTables client, so
 * it connects to the robot the same way Shuffleboard does, and works whether the robot is real or
 * in simulation.
 *
 * <p>Launch with {@code ./gradlew dashboard}, or build {@code ./gradlew dashboardJar} for
 * something double-clickable.
 */
public class DashboardApp {

  private static final Color BG = new Color(0x13, 0x15, 0x19);
  private static final Color PANEL = new Color(0x1C, 0x1F, 0x26);
  private static final Color EDGE = new Color(0x2C, 0x31, 0x3A);
  private static final Color TEXT = new Color(0xD8, 0xDC, 0xE2);
  private static final Color DIM = new Color(0x8A, 0x90, 0x98);
  private static final Color GOOD = new Color(0x3D, 0xDC, 0x84);
  private static final Color BAD = new Color(0xFF, 0x6B, 0x6B);
  private static final Color ACCENT = new Color(0x4C, 0xC2, 0xFF);
  private static final Color WARN = new Color(0xFF, 0xC4, 0x5C);

  /** Default team number; overridden by the first CLI argument. */
  private static final int DEFAULT_TEAM = 7064;

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private NetworkTable dash;
  private NetworkTable controls;

  private final MusicPlayer player = new MusicPlayer();
  private final Map<String, JLabel> valueLabels = new LinkedHashMap<>();

  private FieldView fieldView;
  private CameraView cameraView;
  private JLabel connectionLabel;
  private JLabel nowPlayingLabel;
  private DefaultListModel<MusicPlayer.Track> playlistModel;
  private JComboBox<Integer> driverPortBox;
  private JComboBox<Integer> operatorPortBox;
  private JLabel portsPresentLabel;
  private JComboBox<String> profileBox;
  private ControllerDiagram driverDiagram;
  private ControllerDiagram operatorDiagram;
  private BindingEditor bindingEditor;
  private boolean layoutLoaded = false;

  public static void main(String[] args) {
    int team = DEFAULT_TEAM;
    if (args.length > 0) {
      try {
        team = Integer.parseInt(args[0]);
      } catch (NumberFormatException ignored) {
        // Fall through to the default; a bad argument should not stop the dashboard opening.
      }
    }
    // NetworkTables is a native library. The gradle `dashboard` task extracts the desktop DLLs
    // and points java.library.path at them, which is the normal path. This call is the fallback
    // for running the jar directly, where no one has set that up; it is expected to fail in the
    // gradle case and that failure is harmless, so it stays quiet.
    try {
      edu.wpi.first.util.CombinedRuntimeLoader.loadLibraries(
          DashboardApp.class, "wpiutiljni", "ntcorejni");
    } catch (Throwable ignored) {
      // Already on java.library.path, or genuinely missing -- the NT calls below will say so.
    }

    final int resolvedTeam = team;
    SwingUtilities.invokeLater(() -> new DashboardApp().start(resolvedTeam));
  }

  private void start(int team) {
    try {
      UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
    } catch (Exception ignored) {
      // The default look and feel is fine.
    }

    nt.startClient4("Dashboard");
    nt.setServerTeam(team);
    nt.startDSClient();
    dash = nt.getTable("Dashboard");
    controls = nt.getTable("Controls");

    JFrame frame = new JFrame("Team " + team + " Dashboard");
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.setMinimumSize(new Dimension(1100, 700));
    frame.getContentPane().setBackground(BG);
    frame.setLayout(new BorderLayout());

    frame.add(buildHeader(), BorderLayout.NORTH);

    JTabbedPane tabs = new JTabbedPane();
    tabs.addTab("Match", buildMatchTab());
    tabs.addTab("Controls", buildControlsTab());
    tabs.addTab("Music", buildMusicTab());
    frame.add(tabs, BorderLayout.CENTER);

    loadDefaultSongs();

    // One timer drives every refresh. Fast enough to feel live, slow enough to stay cheap.
    new Timer(50, e -> refresh()).start();

    frame.setVisible(true);
  }

  // ---- Header ---------------------------------------------------------------

  private JPanel buildHeader() {
    JPanel header = new JPanel(new BorderLayout());
    header.setBackground(PANEL);
    header.setBorder(BorderFactory.createEmptyBorder(8, 12, 8, 12));

    connectionLabel = new JLabel("connecting...");
    connectionLabel.setForeground(DIM);
    connectionLabel.setFont(connectionLabel.getFont().deriveFont(Font.BOLD, 13f));
    header.add(connectionLabel, BorderLayout.WEST);

    nowPlayingLabel = new JLabel(" ");
    nowPlayingLabel.setForeground(DIM);
    header.add(nowPlayingLabel, BorderLayout.EAST);
    return header;
  }

  // ---- Match tab ------------------------------------------------------------

  private JPanel buildMatchTab() {
    JPanel root = new JPanel(new BorderLayout(8, 8));
    root.setBackground(BG);
    root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

    fieldView = new FieldView();
    cameraView = new CameraView();

    JSplitPane views = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, fieldView, cameraView);
    views.setResizeWeight(0.62);
    views.setBorder(null);

    root.add(views, BorderLayout.CENTER);
    root.add(buildValuesPanel(), BorderLayout.SOUTH);
    return root;
  }

  /** The short list worth looking at mid-match. Deliberately not forty numbers. */
  private JPanel buildValuesPanel() {
    JPanel grid = new JPanel(new GridLayout(2, 5, 8, 6));
    grid.setBackground(BG);
    grid.setBorder(BorderFactory.createEmptyBorder(8, 0, 0, 0));

    grid.add(valueTile("READY", "ReadyToShoot"));
    grid.add(valueTile("Vision", "Vision/Status"));
    grid.add(valueTile("Range ft", "Shooter/DistanceFeet"));
    grid.add(valueTile("Shooter RPM", "Shooter/ActualRPM"));
    grid.add(valueTile("Turret deg", "Turret/AngleDegrees"));

    grid.add(valueTile("Match", "MatchTime"));
    grid.add(valueTile("Battery", "BatteryVolts"));
    grid.add(valueTile("Tags seen", "Vision/VisibleTags"));
    grid.add(valueTile("Cmd RPM", "Shooter/CommandedRPM"));
    grid.add(valueTile("On target", "Turret/OnTarget"));
    return grid;
  }

  private JPanel valueTile(String caption, String key) {
    JPanel tile = new JPanel(new BorderLayout()) {
      @Override
      protected void paintComponent(java.awt.Graphics g) {
        java.awt.Graphics2D g2 = (java.awt.Graphics2D) g.create();
        g2.setRenderingHint(java.awt.RenderingHints.KEY_ANTIALIASING,
            java.awt.RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setColor(PANEL);
        g2.fillRoundRect(0, 0, getWidth(), getHeight(), 10, 10);
        g2.setColor(EDGE);
        g2.drawRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 10, 10);
        g2.dispose();
      }
    };
    tile.setOpaque(false);
    tile.setBorder(BorderFactory.createEmptyBorder(7, 12, 7, 12));

    JLabel captionLabel = new JLabel(caption.toUpperCase());
    captionLabel.setForeground(DIM);
    captionLabel.setFont(new Font("Segoe UI", Font.BOLD, 10));

    JLabel value = new JLabel("-");
    value.setForeground(TEXT);
    value.setFont(new Font("Consolas", Font.BOLD, 19));

    tile.add(captionLabel, BorderLayout.NORTH);
    tile.add(value, BorderLayout.CENTER);
    valueLabels.put(key, value);
    return tile;
  }

  // ---- Controls tab ---------------------------------------------------------

  private JPanel buildControlsTab() {
    JPanel root = new JPanel();
    root.setLayout(new BoxLayout(root, BoxLayout.Y_AXIS));
    root.setBackground(BG);
    root.setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));

    root.add(heading("Controller ports"));
    root.add(note("Which USB port each role reads from. Change these to swap controllers "
        + "without unplugging anything."));

    driverPortBox = portSelector();
    operatorPortBox = portSelector();

    root.add(labelledRow("Driver", driverPortBox));
    root.add(labelledRow("Operator", operatorPortBox));

    JButton swap = new JButton("Swap driver and operator");
    swap.addActionListener(e -> {
      int driver = (int) driverPortBox.getSelectedItem();
      int operator = (int) operatorPortBox.getSelectedItem();
      controls.getEntry("DriverPort").setDouble(operator);
      controls.getEntry("OperatorPort").setDouble(driver);
    });
    root.add(Box.createVerticalStrut(8));
    root.add(swap);

    portsPresentLabel = new JLabel("ports: -");
    portsPresentLabel.setForeground(DIM);
    root.add(Box.createVerticalStrut(12));
    root.add(portsPresentLabel);

    root.add(Box.createVerticalStrut(24));
    root.add(heading("Binding map"));
    root.add(note("The active control set. Edit the profile name to switch sets; the robot "
        + "reads it live."));

    profileBox = new JComboBox<>();
    profileBox.setMaximumSize(new Dimension(220, 28));
    profileBox.addActionListener(e -> {
      if (profileBox.hasFocus() && profileBox.getSelectedItem() != null) {
        controls.getEntry("Profile").setString(profileBox.getSelectedItem().toString());
      }
    });
    root.add(labelledRow("Profile", profileBox));

    root.add(Box.createVerticalStrut(16));
    root.add(heading("Bindings"));
    root.add(note("Click a button on a controller to select it. Pick an action, choose where it "
        + "lives, and Apply -- the robot re-points that binding immediately, no redeploy."));

    driverDiagram = new ControllerDiagram();
    driverDiagram.setTitle("DRIVER");
    operatorDiagram = new ControllerDiagram();
    operatorDiagram.setTitle("OPERATOR");

    bindingEditor = new BindingEditor(controls);

    driverDiagram.setOnSelect(input -> operatorDiagram.setBindings(
        bindingEditor.bindingsForRole("OPERATOR")));

    JPanel diagrams = new JPanel(new java.awt.GridLayout(1, 2, 8, 0));
    diagrams.setBackground(BG);
    diagrams.add(driverDiagram);
    diagrams.add(operatorDiagram);
    diagrams.setAlignmentX(JPanel.LEFT_ALIGNMENT);
    root.add(diagrams);

    root.add(Box.createVerticalStrut(10));
    bindingEditor.setAlignmentX(JPanel.LEFT_ALIGNMENT);
    root.add(bindingEditor);

    root.add(Box.createVerticalGlue());
    return root;
  }

  private JComboBox<Integer> portSelector() {
    JComboBox<Integer> box = new JComboBox<>(new Integer[] {0, 1, 2, 3, 4, 5});
    box.setMaximumSize(new Dimension(80, 28));
    box.addActionListener(e -> {
      if (box.hasFocus()) {
        String key = box == driverPortBox ? "DriverPort" : "OperatorPort";
        controls.getEntry(key).setDouble((Integer) box.getSelectedItem());
      }
    });
    return box;
  }

  // ---- Music tab ------------------------------------------------------------

  private JPanel buildMusicTab() {
    JPanel root = new JPanel(new BorderLayout(12, 12));
    root.setBackground(BG);
    root.setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));

    playlistModel = new DefaultListModel<>();
    JList<MusicPlayer.Track> list = new JList<>(playlistModel);
    list.setBackground(PANEL);
    list.setForeground(TEXT);
    JScrollPane scroll = new JScrollPane(list);
    scroll.setBorder(BorderFactory.createLineBorder(PANEL));
    root.add(scroll, BorderLayout.CENTER);

    JPanel controlsRow = new JPanel();
    controlsRow.setLayout(new BoxLayout(controlsRow, BoxLayout.X_AXIS));
    controlsRow.setBackground(BG);

    JButton playPause = new JButton("Play");
    playPause.addActionListener(e -> {
      player.togglePause();
      playPause.setText(player.isPlaying() ? "Pause" : "Play");
    });

    JButton next = new JButton("Next");
    next.addActionListener(e -> player.next());

    JButton stop = new JButton("Stop");
    stop.addActionListener(e -> {
      player.stop();
      playPause.setText("Play");
    });

    JCheckBox shuffle = new JCheckBox("Shuffle", true);
    shuffle.setBackground(BG);
    shuffle.setForeground(TEXT);
    shuffle.addActionListener(e -> player.setShuffle(shuffle.isSelected()));

    JSlider volume = new JSlider(0, 100, (int) (player.getVolume() * 100));
    volume.setBackground(BG);
    volume.setMaximumSize(new Dimension(180, 28));
    volume.addChangeListener(e -> player.setVolume(volume.getValue() / 100f));

    JButton add = new JButton("Add songs...");
    add.addActionListener(e -> chooseSongs());

    JButton remove = new JButton("Remove");
    remove.addActionListener(e -> {
      int index = list.getSelectedIndex();
      if (index >= 0) {
        player.removeTrack(index);
        refreshPlaylist();
      }
    });

    controlsRow.add(playPause);
    controlsRow.add(Box.createHorizontalStrut(6));
    controlsRow.add(next);
    controlsRow.add(Box.createHorizontalStrut(6));
    controlsRow.add(stop);
    controlsRow.add(Box.createHorizontalStrut(14));
    controlsRow.add(shuffle);
    controlsRow.add(Box.createHorizontalStrut(14));
    controlsRow.add(new JLabel("Vol"));
    controlsRow.add(volume);
    controlsRow.add(Box.createHorizontalGlue());
    controlsRow.add(add);
    controlsRow.add(Box.createHorizontalStrut(6));
    controlsRow.add(remove);

    JPanel south = new JPanel(new BorderLayout(0, 8));
    south.setBackground(BG);
    south.add(new Visualizer(player), BorderLayout.NORTH);
    south.add(controlsRow, BorderLayout.SOUTH);
    root.add(south, BorderLayout.SOUTH);

    player.setOnTrackChanged(track -> SwingUtilities.invokeLater(() ->
        nowPlayingLabel.setText("♪  " + track.title)));
    player.setOnError(message -> SwingUtilities.invokeLater(() ->
        nowPlayingLabel.setText(message)));

    return root;
  }

  private void chooseSongs() {
    JFileChooser chooser = new JFileChooser();
    chooser.setMultiSelectionEnabled(true);
    chooser.setFileFilter(new FileNameExtensionFilter("MP3 audio", "mp3"));
    if (chooser.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
      for (File file : chooser.getSelectedFiles()) {
        player.addTrack(file);
      }
      refreshPlaylist();
    }
  }

  /** Picks up any MP3s sitting next to the project or in Downloads, so it is not empty on day one. */
  private void loadDefaultSongs() {
    File[] candidates = {
        new File("music"),
        new File(System.getProperty("user.home"), "Downloads")
    };
    for (File dir : candidates) {
      File[] found = dir.isDirectory()
          ? dir.listFiles(f -> f.getName().toLowerCase().endsWith(".mp3"))
          : null;
      if (found != null) {
        for (File file : found) {
          player.addTrack(file);
        }
      }
    }
    refreshPlaylist();
  }

  private void refreshPlaylist() {
    playlistModel.clear();
    player.playlist().forEach(playlistModel::addElement);
  }

  // ---- Refresh --------------------------------------------------------------

  private void refresh() {
    boolean connected = nt.isConnected();
    connectionLabel.setText(connected ? "ROBOT CONNECTED" : "robot not connected");
    connectionLabel.setForeground(connected ? GOOD : BAD);

    for (Map.Entry<String, JLabel> entry : valueLabels.entrySet()) {
      JLabel label = entry.getValue();
      label.setText(readAsText(entry.getKey()));
      label.setForeground(colourFor(entry.getKey()));
    }

    if (!layoutLoaded) {
      double[] tags = dash.getEntry("Field/Tags").getDoubleArray(new double[0]);
      if (tags.length > 0) {
        fieldView.setFieldSize(
            dash.getEntry("Field/LengthMeters").getDouble(16.541),
            dash.getEntry("Field/WidthMeters").getDouble(8.069));
        fieldView.setTagLayout(tags);
        layoutLoaded = true;
      }
    }

    fieldView.setVisibleTags(dash.getEntry("Vision/VisibleTagIds").getDoubleArray(new double[0]));
    fieldView.setRobotPose(
        dash.getEntry("Vision/PoseValid").getBoolean(false),
        dash.getEntry("Vision/EstimatedX").getDouble(0),
        dash.getEntry("Vision/EstimatedY").getDouble(0),
        dash.getEntry("Vision/EstimatedHeadingDegrees").getDouble(0));
    fieldView.repaint();

    cameraView.setCandidates(buildCameraCandidates());

    // Only follow the robot's port values when the user is not editing the boxes.
    if (!driverPortBox.hasFocus()) {
      driverPortBox.setSelectedItem((int) controls.getEntry("DriverPort").getDouble(0));
    }
    if (!operatorPortBox.hasFocus()) {
      operatorPortBox.setSelectedItem((int) controls.getEntry("OperatorPort").getDouble(1));
    }
    portsPresentLabel.setText("ports: " + controls.getEntry("PortsPresent").getString("-"));

    String[] options = controls.getEntry("ProfileOptions").getStringArray(new String[0]);
    if (options.length != profileBox.getItemCount()) {
      profileBox.removeAllItems();
      for (String option : options) {
        profileBox.addItem(option);
      }
    }
    if (!profileBox.hasFocus()) {
      String active = controls.getEntry("ActiveProfile").getString("");
      if (!active.isEmpty()) {
        profileBox.setSelectedItem(active);
      }
    }
    bindingEditor.update(
        controls.getEntry("ActionOptions").getStringArray(new String[0]),
        controls.getEntry("InputOptions").getStringArray(new String[0]),
        controls.getEntry("ActiveMap").getString(""));
    driverDiagram.setBindings(bindingEditor.bindingsForRole("DRIVER"));
    operatorDiagram.setBindings(bindingEditor.bindingsForRole("OPERATOR"));
  }

  /**
   * Every address the camera stream might be reachable at, best first.
   *
   * <p>The roboRIO's forwarded ports come first. The camera's own IP is often unreachable from
   * here — plugged into the roboRIO over USB it sits on 172.29.0.1, a private link between those
   * two devices that this laptop has no route to. The robot forwards that link out on its own
   * ports, so going via the robot works regardless of how the camera is cabled.
   */
  private java.util.List<String> buildCameraCandidates() {
    java.util.List<String> urls = new java.util.ArrayList<>();

    String robotIp = null;
    var connections = nt.getConnections();
    if (connections.length > 0) {
      robotIp = connections[0].remote_ip;
    }

    if (robotIp != null) {
      // 5800 mDNS forward, 5820 USB forward, 5830 static-IP forward. 5810 is NT4.
      urls.add("http://" + robotIp + ":5800/stream.mjpg");
      urls.add("http://" + robotIp + ":5820/stream.mjpg");
      urls.add("http://" + robotIp + ":5830/stream.mjpg");
    }

    // Whatever the robot thinks, last: it reports the camera's own address, which only helps on
    // the rare occasion this laptop shares a subnet with it.
    String hint = dash.getEntry("Vision/StreamUrl").getString("");
    if (!hint.isBlank()) {
      urls.add(hint);
    }
    urls.add("http://limelight.local:5800/stream.mjpg");
    return urls;
  }

  /**
   * Colour for a value, so the tiles that matter read without being parsed.
   *
   * <p>Only the few things worth reacting to mid-match get colour; everything else stays neutral.
   * A dashboard where every number is coloured is a dashboard where none of them stand out.
   */
  private Color colourFor(String key) {
    switch (key) {
      case "ReadyToShoot":
        return dash.getEntry(key).getBoolean(false) ? GOOD : DIM;

      case "Turret/OnTarget":
        return dash.getEntry(key).getBoolean(false) ? GOOD : WARN;

      case "Vision/Status": {
        String status = dash.getEntry(key).getString("");
        if (status.startsWith("LOCKED")) {
          return GOOD;
        }
        return status.contains("DISCONNECT") ? BAD : WARN;
      }

      case "BatteryVolts": {
        double volts = dash.getEntry(key).getDouble(0);
        if (volts <= 0) {
          return DIM;
        }
        // Below 11 V under load the roboRIO is heading for a brownout.
        return volts < 11.0 ? BAD : (volts < 12.0 ? WARN : GOOD);
      }

      case "MatchTime": {
        double seconds = dash.getEntry(key).getDouble(-1);
        if (seconds < 0) {
          return DIM;
        }
        return seconds <= 20.0 ? WARN : TEXT;
      }

      default:
        return TEXT;
    }
  }

  /** Reads a key without needing to know its type. */
  private String readAsText(String key) {
    var entry = dash.getEntry(key);
    switch (entry.getType()) {
      case kBoolean:
        return entry.getBoolean(false) ? "YES" : "no";
      case kDouble:
        double v = entry.getDouble(0);
        return Math.abs(v) >= 100 ? String.format("%.0f", v) : String.format("%.2f", v);
      case kString:
        return entry.getString("-");
      default:
        return "-";
    }
  }

  private JLabel heading(String text) {
    JLabel label = new JLabel(text);
    label.setForeground(ACCENT);
    label.setFont(label.getFont().deriveFont(Font.BOLD, 14f));
    label.setAlignmentX(JPanel.LEFT_ALIGNMENT);
    return label;
  }

  private JLabel note(String text) {
    JLabel label = new JLabel("<html><body style='width:520px'>" + text + "</body></html>");
    label.setForeground(DIM);
    label.setAlignmentX(JPanel.LEFT_ALIGNMENT);
    label.setBorder(BorderFactory.createEmptyBorder(2, 0, 8, 0));
    return label;
  }

  private JPanel labelledRow(String caption, java.awt.Component field) {
    JPanel row = new JPanel();
    row.setLayout(new BoxLayout(row, BoxLayout.X_AXIS));
    row.setBackground(BG);
    row.setAlignmentX(JPanel.LEFT_ALIGNMENT);
    row.setMaximumSize(new Dimension(420, 32));

    JLabel label = new JLabel(caption);
    label.setForeground(TEXT);
    label.setPreferredSize(new Dimension(90, 24));
    row.add(label);
    row.add(field);
    row.add(Box.createHorizontalGlue());
    return row;
  }

}
