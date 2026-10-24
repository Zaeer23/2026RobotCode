package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.GraphicsEnvironment;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.Rectangle;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTabbedPane;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

import com.formdev.flatlaf.FlatClientProperties;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Driver station dashboard.
 *
 * <p>Runs on the laptop, not the robot — the roboRIO is headless. It is a NetworkTables client, so
 * it connects to the robot the same way Shuffleboard does, and works whether the robot is real or
 * in simulation.
 *
 * <p>Four tabs: <b>Match</b> (field, camera, and the handful of numbers worth watching),
 * <b>Autonomous</b> (pick the routine, with a picture of what it does), <b>Controls</b> (ports
 * and live rebinding) and <b>Music</b> (searchable songs and playlists, see {@link MusicTab}).
 * The header across the top is the same on every tab.
 *
 * <p>Launch with {@code ./gradlew dashboard}, or build {@code ./gradlew dashboardJar} for
 * something double-clickable. The first argument is the team number, or a host name such as
 * {@code localhost} to connect to a simulated robot.
 */
public class DashboardApp {

  /** Default team number; overridden by the first CLI argument. */
  private static final int DEFAULT_TEAM = 7064;

  /** Must match {@code AutoCatalog.CHOOSER_KEY} on the robot. */
  private static final String AUTO_CHOOSER = "Auto Routine";

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private NetworkTable dash;
  private NetworkTable controls;
  private NetworkTable autoChooser;
  private final PsychoOverlay psychoOverlay = new PsychoOverlay();
  private PsychoArmPanel psychoArm;

  /** Fills the window with just the camera once AUTO is done in CYBERPSYCHO. */
  private final JPanel cameraOnlyPane = new JPanel(new BorderLayout());
  private boolean showingCameraOnly = false;
  private JFrame frame;
  private JTabbedPane tabs;

  private final MusicPlayer player = new MusicPlayer();

  private HeaderBar header;
  private AutoTab autoTab;

  // Match tab.
  private FieldView fieldView;
  private CameraView cameraView;
  private Pill fixPill;
  private ReadyTile readyTile;
  private StatTile visionTile;
  private StatTile rangeTile;
  private StatTile shooterTile;
  private StatTile turretTile;

  // Controls tab.
  private JComboBox<Integer> driverPortBox;
  private JComboBox<Integer> operatorPortBox;
  private JLabel portsPresentLabel;
  private JComboBox<String> profileBox;
  private ControllerDiagram driverDiagram;
  private ControllerDiagram operatorDiagram;
  private BindingEditor bindingEditor;

  private Pill nowPlayingChip;

  private boolean layoutLoaded = false;

  public static void main(String[] args) {
    int team = DEFAULT_TEAM;
    String host = null;
    if (args.length > 0) {
      try {
        team = Integer.parseInt(args[0].trim());
      } catch (NumberFormatException notATeam) {
        // Not a number, so a server address: "localhost" for a simulated robot.
        host = args[0].trim();
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
    final String resolvedHost = host;
    SwingUtilities.invokeLater(() -> new DashboardApp().start(resolvedTeam, resolvedHost));
  }

  private void start(int team, String host) {
    Theme.install();

    // The offline auto previews use PathPlanner, which posts its own alerts through SmartDashboard.
    // Pointed at the robot's connection, those would appear on the robot's NetworkTables as if the
    // robot had raised them. The dashboard uses nothing else of SmartDashboard, so it gets a
    // private, unconnected instance.
    SmartDashboard.setNetworkTableInstance(NetworkTableInstance.create());

    nt.startClient4("Dashboard");
    if (host != null) {
      nt.setServer(host);
    } else {
      nt.setServerTeam(team);
      nt.startDSClient();
    }
    dash = nt.getTable("Dashboard");
    controls = nt.getTable("Controls");
    autoChooser = nt.getTable("SmartDashboard").getSubTable(AUTO_CHOOSER);

    frame = new JFrame("Team " + team + "  ·  Driver Dashboard");
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.setIconImages(HeaderBar.Badge.windowIcons(team));
    frame.setMinimumSize(new Dimension(1100, 680));
    frame.getContentPane().setBackground(Theme.BG);
    frame.setLayout(new BorderLayout());

    header = new HeaderBar(team);
    frame.add(header, BorderLayout.NORTH);

    tabs = new JTabbedPane();
    tabs.addTab("Match", Icons.match(16, Theme.TEXT_DIM), buildMatchTab());
    autoTab = new AutoTab(label -> autoChooser.getEntry("selected").setString(label));
    tabs.addTab("Autonomous", Icons.route(16, Theme.TEXT_DIM), autoTab);
    tabs.addTab("Controls", Icons.gamepad(16, Theme.TEXT_DIM), buildControlsTab());
    tabs.addTab("Music", Icons.music(16, Theme.TEXT_DIM), new MusicTab(player));
    tabs.putClientProperty(FlatClientProperties.TABBED_PANE_TRAILING_COMPONENT, buildTabTrailer());
    frame.add(tabs, BorderLayout.CENTER);
    cameraOnlyPane.setOpaque(true);
    cameraOnlyPane.setBackground(Theme.SURFACE);
    cameraOnlyPane.setVisible(false);
    frame.add(cameraOnlyPane, BorderLayout.SOUTH);

    // One timer drives every refresh. Fast enough to feel live, slow enough to stay cheap.
    loadOfflineField();
    new Thread(() -> {
      OfflineAutos.Result offline = OfflineAutos.load();
      SwingUtilities.invokeLater(() -> autoTab.setOfflineCatalog(offline.catalogJson(),
          offline.problem()));
    }, "offline-autos").start();

    new Timer(50, e -> refresh()).start();
    refresh();

    Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
    frame.setSize(Math.min(1480, screen.width), Math.min(920, screen.height));
    frame.setLocationRelativeTo(null);
    // Glass pane so CYBERPSYCHO paints over everything without taking any clicks.
    frame.setGlassPane(psychoOverlay);
    psychoOverlay.setVisible(false);

    frame.setVisible(true);
  }

  /**
   * The official field layout, so the field and the auto previews draw properly before a robot is
   * connected. The robot publishes the same layout once it is, which replaces this.
   */
  private void loadOfflineField() {
    try {
      AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      double[] tags = new double[layout.getTags().size() * 5];
      int i = 0;
      for (AprilTag tag : layout.getTags()) {
        tags[i++] = tag.ID;
        tags[i++] = tag.pose.getX();
        tags[i++] = tag.pose.getY();
        tags[i++] = tag.pose.getZ();
        tags[i++] = Math.toDegrees(tag.pose.getRotation().getZ());
      }
      fieldView.setFieldSize(layout.getFieldLength(), layout.getFieldWidth());
      fieldView.setTagLayout(tags);
      autoTab.setFieldLayout(layout.getFieldLength(), layout.getFieldWidth(), tags);
    } catch (Exception e) {
      // Without it the field still draws from its built-in dimensions, just without tags.
    }
  }

  /** Now-playing chip at the right end of the tab strip, so the music never needs its own tab. */
  private JComponent buildTabTrailer() {
    JPanel trailer = new JPanel(new FlowLayout(FlowLayout.RIGHT, 0, 9));
    trailer.setOpaque(false);
    trailer.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 8));
    nowPlayingChip = new Pill("", Theme.ROUTE_INTAKE);
    nowPlayingChip.setVisible(false);
    trailer.add(nowPlayingChip);
    return trailer;
  }

  // ---- Match tab ----------------------------------------------------------------------------------

  private JPanel buildMatchTab() {
    JPanel root = new JPanel(new BorderLayout(0, Theme.GAP));
    root.setBackground(Theme.BG);
    root.setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));

    fieldView = new FieldView();
    cameraView = new CameraView();

    fixPill = new Pill("NO FIX", Theme.TEXT_FAINT);
    Card fieldCard = new Card("Field", "Position from AprilTags, no odometry");
    fieldCard.setPadding(14, 16, 12, 16);
    fieldCard.addHeaderRight(fixPill);
    fieldCard.content(fieldView);

    Card cameraCard = new Card("Limelight", "Forwarded through the robot");
    cameraCard.setPadding(14, 16, 16, 16);
    cameraCard.content(cameraView);

    psychoArm = new PsychoArmPanel(dash);
    Card psychoCard = new Card("CYBERPSYCHO",
        "All power to the wheels, no mechanisms, no way back once AUTO starts");
    psychoCard.setPadding(12, 16, 12, 16);
    psychoCard.content(psychoArm);

    JSplitPane views = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, fieldCard, cameraCard);
    views.setResizeWeight(0.64);
    views.setBorder(null);
    views.setOpaque(false);
    views.setContinuousLayout(true);
    root.add(views, BorderLayout.CENTER);
    root.add(psychoCard, BorderLayout.SOUTH);
    root.add(buildTiles(), BorderLayout.SOUTH);
    return root;
  }

  /** The short list worth looking at mid-match. Deliberately not forty numbers. */
  private JPanel buildTiles() {
    readyTile = new ReadyTile();
    visionTile = new StatTile("Vision");
    rangeTile = new StatTile("Range to hub");
    shooterTile = new StatTile("Shooter");
    turretTile = new StatTile("Turret");

    JPanel row = new JPanel(new GridBagLayout());
    row.setOpaque(false);
    GridBagConstraints c = new GridBagConstraints();
    c.fill = GridBagConstraints.BOTH;
    c.weighty = 1.0;
    c.gridy = 0;

    Component[] tiles = {readyTile, visionTile, rangeTile, shooterTile, turretTile};
    double[] weights = {2.2, 1.3, 1.0, 1.2, 1.0};
    for (int i = 0; i < tiles.length; i++) {
      c.gridx = i;
      c.weightx = weights[i];
      c.insets = new Insets(0, i == 0 ? 0 : Theme.GAP, 0, 0);
      row.add(tiles[i], c);
    }
    row.setPreferredSize(new Dimension(10, 110));
    return row;
  }

  // ---- Controls tab -------------------------------------------------------------------------------

  private JComponent buildControlsTab() {
    JPanel root = new TrackingPanel(new BorderLayout(Theme.GAP, 0));
    root.setBackground(Theme.BG);
    root.setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));

    // Ports.
    driverPortBox = portSelector();
    operatorPortBox = portSelector();

    JButton swap = Ui.secondary("Swap driver and operator", Icons.swap(14, Theme.TEXT));
    swap.addActionListener(e -> {
      int driver = (int) driverPortBox.getSelectedItem();
      int operator = (int) operatorPortBox.getSelectedItem();
      controls.getEntry("DriverPort").setDouble(operator);
      controls.getEntry("OperatorPort").setDouble(driver);
    });
    portsPresentLabel = Ui.label("-", Theme.numeric(Font.PLAIN, 14f), Theme.TEXT);

    JPanel portsForm = form(
        new Object[] {"Driver", driverPortBox},
        new Object[] {"Operator", operatorPortBox},
        new Object[] {"Plugged in", portsPresentLabel});
    JPanel portsBody = new JPanel(new BorderLayout(0, 12));
    portsBody.setOpaque(false);
    portsBody.add(portsForm, BorderLayout.CENTER);
    JPanel swapRow = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
    swapRow.setOpaque(false);
    swapRow.add(swap);
    portsBody.add(swapRow, BorderLayout.SOUTH);

    Card ports = new Card("Controller ports", "Which USB port each role reads from");
    ports.content(portsBody);

    // Profile.
    profileBox = new JComboBox<>();
    profileBox.setPreferredSize(new Dimension(200, 32));
    profileBox.addActionListener(e -> {
      if (profileBox.hasFocus() && profileBox.getSelectedItem() != null) {
        controls.getEntry("Profile").setString(profileBox.getSelectedItem().toString());
      }
    });
    Card profile = new Card("Binding map", "The active control set, read live by the robot");
    profile.content(form(new Object[] {"Profile", profileBox}));

    JPanel side = new JPanel();
    side.setLayout(new BoxLayout(side, BoxLayout.Y_AXIS));
    side.setOpaque(false);
    ports.setAlignmentX(Component.LEFT_ALIGNMENT);
    profile.setAlignmentX(Component.LEFT_ALIGNMENT);
    ports.setMaximumSize(new Dimension(Integer.MAX_VALUE, ports.getPreferredSize().height));
    profile.setMaximumSize(new Dimension(Integer.MAX_VALUE, profile.getPreferredSize().height));
    side.add(ports);
    side.add(Box.createVerticalStrut(Theme.GAP));
    side.add(profile);
    side.add(Box.createVerticalGlue());
    side.setPreferredSize(new Dimension(340, 10));

    // Bindings.
    driverDiagram = new ControllerDiagram();
    driverDiagram.setTitle("DRIVER");
    operatorDiagram = new ControllerDiagram();
    operatorDiagram.setTitle("OPERATOR");

    bindingEditor = new BindingEditor(controls);

    // Clicking a button on either pad points the editor at it and clears the selection on the
    // other pad, so only one target is ever highlighted. This used to set the OPERATOR diagram's
    // bindings from a DRIVER click and nothing else, which meant the drawing was not actually
    // wired to the editor at all.
    driverDiagram.setOnSelect(input -> {
      operatorDiagram.setSelectedInput(null);
      bindingEditor.selectTarget("DRIVER", input);
    });
    operatorDiagram.setOnSelect(input -> {
      driverDiagram.setSelectedInput(null);
      bindingEditor.selectTarget("OPERATOR", input);
    });

    JPanel diagrams = new JPanel(new GridLayout(1, 2, Theme.GAP, 0));
    diagrams.setOpaque(false);
    diagrams.add(diagramWell(driverDiagram));
    diagrams.add(diagramWell(operatorDiagram));

    JPanel bindingsBody = new JPanel(new BorderLayout(0, 16));
    bindingsBody.setOpaque(false);
    bindingsBody.add(diagrams, BorderLayout.CENTER);
    bindingsBody.add(bindingEditor, BorderLayout.SOUTH);

    Card bindings = new Card("Bindings",
        "Click a button on a controller to select it. Pick an action, choose where it lives, and "
            + "Apply: the robot re-points that binding immediately, no redeploy.");
    bindings.content(bindingsBody);

    root.add(side, BorderLayout.WEST);
    root.add(bindings, BorderLayout.CENTER);

    JScrollPane scroll = new JScrollPane(root);
    scroll.setBorder(null);
    scroll.getVerticalScrollBar().setUnitIncrement(16);
    scroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    return scroll;
  }

  /** A recessed panel behind each controller drawing, so the pad sits on something. */
  private static JComponent diagramWell(ControllerDiagram diagram) {
    JPanel well = new JPanel(new BorderLayout()) {
      @Override
      protected void paintComponent(java.awt.Graphics g) {
        java.awt.Graphics2D g2 = (java.awt.Graphics2D) g.create();
        Theme.smooth(g2);
        g2.setColor(Theme.SURFACE_RAISED);
        g2.fill(new java.awt.geom.RoundRectangle2D.Double(0, 0, getWidth(), getHeight(), 12, 12));
        g2.dispose();
      }
    };
    well.setOpaque(false);
    well.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
    well.add(diagram, BorderLayout.CENTER);
    well.setPreferredSize(new Dimension(480, 250));
    return well;
  }

  /** Label/field rows, labels aligned in one column. Each row is {caption, component}. */
  private static JPanel form(Object[]... rows) {
    JPanel form = new JPanel(new GridBagLayout());
    form.setOpaque(false);
    GridBagConstraints c = new GridBagConstraints();
    c.gridy = 0;
    c.anchor = GridBagConstraints.WEST;
    for (Object[] row : rows) {
      c.gridx = 0;
      c.weightx = 0;
      c.fill = GridBagConstraints.NONE;
      c.insets = new Insets(0, 0, 8, 16);
      form.add(Ui.label((String) row[0], Theme.ui(Font.PLAIN, 13f), Theme.TEXT_DIM), c);
      c.gridx = 1;
      c.weightx = 1;
      c.insets = new Insets(0, 0, 8, 0);
      JPanel holder = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
      holder.setOpaque(false);
      holder.add((Component) row[1]);
      form.add(holder, c);
      c.gridy++;
    }
    JPanel top = new JPanel(new BorderLayout());
    top.setOpaque(false);
    top.add(form, BorderLayout.NORTH);
    return top;
  }

  private JComboBox<Integer> portSelector() {
    JComboBox<Integer> box = new JComboBox<>(new Integer[] {0, 1, 2, 3, 4, 5});
    box.setPreferredSize(new Dimension(120, 32));
    box.addActionListener(e -> {
      if (box.hasFocus()) {
        String key = box == driverPortBox ? "DriverPort" : "OperatorPort";
        controls.getEntry(key).setDouble((Integer) box.getSelectedItem());
      }
    });
    return box;
  }

  // ---- Refresh ------------------------------------------------------------------------------------

  private void refresh() {
    boolean connected = nt.isConnected();
    var connections = nt.getConnections();
    String robotIp = connections.length > 0 ? connections[0].remote_ip : null;

    boolean enabled = dash.getEntry("Enabled").getBoolean(false);
    boolean psychoActive = dash.getEntry(CyberPsychoKeys.ACTIVE).getBoolean(false);
    psychoArm.update(dash.getEntry(CyberPsychoKeys.ARMED).getBoolean(false), psychoActive);
    psychoOverlay.setActive(psychoArm.shouldDistort());

    // Once AUTO is over in this mode the screen is handed to the camera. Nothing else on it is
    // useful when no mechanism is listening and the driver is going flat out.
    boolean cameraOnly = psychoActive
        && dash.getEntry(CyberPsychoKeys.AUTO_DONE).getBoolean(false);
    if (cameraOnly != showingCameraOnly) {
      showingCameraOnly = cameraOnly;
      tabs.setVisible(!cameraOnly);
      cameraOnlyPane.setVisible(cameraOnly);
      if (cameraOnly) {
        cameraOnlyPane.add(cameraView, BorderLayout.CENTER);
      }
      frame.revalidate();
    }
    String alliance = dash.getEntry("Alliance").getString("");
    String activeAuto = autoChooser.getEntry("active").getString("");

    header.update(
        connected,
        robotIp,
        enabled,
        dash.getEntry("Mode").getString(
            dash.getEntry("Autonomous").getBoolean(false) ? "AUTO" : "TELEOP"),
        alliance,
        dash.getEntry("FMSAttached").getBoolean(false),
        activeAuto,
        dash.getEntry("MatchTime").getDouble(-1),
        dash.getEntry("BatteryVolts").getDouble(0),
        dash.getEntry("Brownout").getBoolean(false));

    if (!layoutLoaded) {
      double[] tags = dash.getEntry("Field/Tags").getDoubleArray(new double[0]);
      if (tags.length > 0) {
        double length = dash.getEntry("Field/LengthMeters").getDouble(16.541);
        double width = dash.getEntry("Field/WidthMeters").getDouble(8.069);
        fieldView.setFieldSize(length, width);
        fieldView.setTagLayout(tags);
        autoTab.setFieldLayout(length, width, tags);
        layoutLoaded = true;
      }
    }

    autoTab.update(
        connected,
        dash.getEntry("Auto/Catalog").getString(""),
        autoChooser.getEntry("options").getStringArray(new String[0]),
        activeAuto,
        connected ? alliance : "");

    refreshMatchTab(connected, enabled, alliance);
    refreshControlsTab();

    MusicPlayer.Track playing = player.currentTrack();
    boolean musicOn = player.isPlaying() && playing != null;
    if (musicOn) {
      nowPlayingChip.set(StatTile.ellipsize(nowPlayingChip.getFontMetrics(nowPlayingChip.getFont()),
          playing.title.toUpperCase(), 240), Theme.ROUTE_INTAKE);
    }
    if (nowPlayingChip.isVisible() != musicOn) {
      nowPlayingChip.setVisible(musicOn);
      nowPlayingChip.getParent().revalidate();
    }
  }

  private void refreshMatchTab(boolean connected, boolean enabled, String alliance) {
    boolean poseValid = dash.getEntry("Vision/PoseValid").getBoolean(false);
    fieldView.setVisibleTags(dash.getEntry("Vision/VisibleTagIds").getDoubleArray(new double[0]));
    fieldView.setRobotPose(
        poseValid,
        dash.getEntry("Vision/EstimatedX").getDouble(0),
        dash.getEntry("Vision/EstimatedY").getDouble(0),
        dash.getEntry("Vision/EstimatedHeadingDegrees").getDouble(0));

    // Before the match, the auto that is about to run is drawn faintly behind the live view as
    // a last check that it is the right one, on the right side.
    AutoRoutine upcoming = connected && !enabled ? autoTab.activeRoutine() : null;
    fieldView.setRoutine(upcoming, "red".equalsIgnoreCase(alliance), true);
    fieldView.repaint();

    fixPill.set(connected && poseValid ? "TAG FIX" : "NO FIX",
        connected && poseValid ? Theme.GOOD : Theme.TEXT_FAINT);

    cameraView.setCandidates(buildCameraCandidates());

    readyTile.update(connected,
        dash.getEntry("ReadyToShoot").getBoolean(false),
        dash.getEntry("Vision/HasTarget").getBoolean(false),
        dash.getEntry("Turret/OnTarget").getBoolean(false),
        dash.getEntry("Shooter/AtSpeed").getBoolean(false));

    if (!connected) {
      for (StatTile tile : new StatTile[] {visionTile, rangeTile, shooterTile, turretTile}) {
        tile.update("-", "", "No robot", Theme.TEXT_FAINT, null, Double.NaN);
      }
      return;
    }

    refreshVisionTile();

    double feet = dash.getEntry("Shooter/DistanceFeet").getDouble(0);
    rangeTile.update(
        feet > 0 ? String.format("%.1f", feet) : "--",
        "ft",
        feet > 0 ? String.format("%.2f m to the hub", feet / 3.28084) : "No hub tag in view",
        feet > 0 ? Theme.TEXT : Theme.TEXT_FAINT,
        null,
        Double.NaN);

    double actual = dash.getEntry("Shooter/ActualRPM").getDouble(0);
    double commanded = dash.getEntry("Shooter/CommandedRPM").getDouble(0);
    boolean atSpeed = dash.getEntry("Shooter/AtSpeed").getBoolean(false);
    shooterTile.update(
        String.format("%.0f", actual),
        "rpm",
        commanded > 0 ? String.format("Target %.0f rpm", commanded) : "Idle",
        atSpeed ? Theme.GOOD : Theme.TEXT,
        atSpeed ? Theme.GOOD : (commanded > 0 ? Theme.WARN : null),
        commanded > 0 ? actual / commanded : Double.NaN);

    boolean onTarget = dash.getEntry("Turret/OnTarget").getBoolean(false);
    double error = dash.getEntry("Turret/ErrorDegrees").getDouble(0);
    turretTile.update(
        String.format("%.1f", dash.getEntry("Turret/AngleDegrees").getDouble(0)),
        "°",
        onTarget ? "On target" : String.format("%.1f° off target", Math.abs(error)),
        onTarget ? Theme.GOOD : Theme.TEXT,
        onTarget ? Theme.GOOD : Theme.WARN,
        Double.NaN);
  }

  /**
   * The camera's own status, shortened to a word with the rest underneath.
   *
   * <p>The robot reports things like "LOCKED ON 20 (BLUE HUB)" or "SEES 2 TAG(S), NONE ELIGIBLE".
   * The headline is what to react to; the detail is for when it is not what you expected.
   */
  private void refreshVisionTile() {
    String status = dash.getEntry("Vision/Status").getString("");
    String visible = dash.getEntry("Vision/VisibleTags").getString("");
    String headline;
    String detail;
    java.awt.Color colour;
    if (status.startsWith("LOCKED")) {
      headline = "LOCKED";
      detail = status.replaceFirst("^LOCKED ON\\s*", "");
      colour = Theme.GOOD;
    } else if (status.contains("DISCONNECT")) {
      headline = "OFFLINE";
      detail = "Camera not connected";
      colour = Theme.BAD;
    } else if (status.contains("NO TAGS")) {
      headline = "SEARCHING";
      detail = "Connected, no tags in view";
      colour = Theme.WARN;
    } else if (status.contains("NONE ELIGIBLE")) {
      headline = "FILTERED";
      detail = status;
      colour = Theme.WARN;
    } else {
      headline = status.isEmpty() ? "-" : status;
      detail = "";
      colour = Theme.TEXT_DIM;
    }
    if (!visible.isEmpty() && !"none".equals(visible) && !visible.startsWith("camera")) {
      detail = "Sees " + visible;
    }
    visionTile.update(headline, "", detail, colour, colour, Double.NaN);
  }

  private void refreshControlsTab() {
    // Only follow the robot's port values when the user is not editing the boxes.
    if (!driverPortBox.hasFocus()) {
      driverPortBox.setSelectedItem((int) controls.getEntry("DriverPort").getDouble(0));
    }
    if (!operatorPortBox.hasFocus()) {
      operatorPortBox.setSelectedItem((int) controls.getEntry("OperatorPort").getDouble(1));
    }
    String present = controls.getEntry("PortsPresent").getString("");
    portsPresentLabel.setText(present.isBlank() ? "-" : present);

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
}
