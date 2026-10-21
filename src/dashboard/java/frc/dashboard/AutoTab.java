package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.Scrollable;
import javax.swing.SwingConstants;
import javax.swing.Timer;

/**
 * Picks the autonomous routine, with a picture of what it will do.
 *
 * <p>Click a routine to preview it: the field shows where the robot starts, the path it drives,
 * every step numbered in order and where each shot is taken from, and a ghost robot plays the
 * routine back in real time. The step list and timeline beside it say the same thing in words and
 * seconds. Nothing reaches the robot until <em>Send to robot</em> (or a double-click), so browsing
 * the options can never change the auto by accident.
 *
 * <p>The robot is the source of truth for what is selected. The panel at the bottom left shows the
 * routine the robot reports it will run, and only says "confirmed" once the robot has echoed the
 * choice back, so a selection lost to a dropped connection is visible before the match.
 *
 * <p>Previews come from the robot itself ({@code Dashboard/Auto/Catalog}), built from the files
 * actually deployed to it. With no robot connected, the tab shows the same previews built from this
 * laptop's copy of the project ({@link OfflineAutos}), marked as such, so routines can be looked at
 * in the pits or at home; they just cannot be sent anywhere. Paths are drawn for whichever alliance the robot reports, mirrored the
 * same way PathPlanner mirrors them; the Blue/Red switch previews the other side without changing
 * anything on the robot.
 */
public class AutoTab extends JPanel {

  /** Pause on the final frame before the preview loops, so the end position can be read. */
  private static final double END_HOLD_SECONDS = 1.6;

  /** How long to wait for the robot to echo a selection before calling it unconfirmed. */
  private static final long CONFIRM_TIMEOUT_NANOS = 2_500_000_000L;

  private final Consumer<String> sendSelection;

  private final FieldView field = new FieldView();
  private final TrackingPanel routineList = new TrackingPanel();
  private final Card previewCard = new Card("Autonomous", " ");
  private final Timeline timeline = new Timeline();
  private final StepList stepList = new StepList();
  private final JLabel timeLabel = Ui.label("0.0 s", Theme.numeric(Font.PLAIN, 13f), Theme.TEXT_DIM);
  private final JButton playButton = Ui.icon(Icons.pause(14, Theme.TEXT), "Pause preview");
  private final JButton restartButton = Ui.icon(Icons.restart(14, Theme.TEXT), "Play from the start");
  private final JButton sendButton = Ui.primary("Send to robot", Icons.send(14, Theme.TEXT_ON_ACCENT));
  private final JLabel activeName = Ui.label("-", Theme.ui(Font.BOLD, 14f), Theme.TEXT);
  private final Pill activePill = new Pill("OFFLINE", Theme.TEXT_FAINT);
  private final Segmented allianceToggle = new Segmented("Blue", "Red");

  private List<AutoRoutine> routines = new ArrayList<>();
  private final Map<String, RoutineCard> cards = new LinkedHashMap<>();
  private String lastCatalog = null;
  private List<String> lastOptions = List.of();
  private String catalogProblem = null;

  /** Routines built from this laptop's project files, shown while no robot is connected. */
  private String offlineCatalog = "";
  private String offlineProblem = "Looking for autos on this laptop\u2026";
  private boolean showingOffline = true;
  private Card routinesCard;
  private final Pill sourcePill = new Pill("FROM LAPTOP", Theme.TEXT_DIM);

  private String previewLabel;
  private String active = "";
  private boolean connected = false;
  private String robotAlliance = "";
  private Boolean allianceOverrideRed = null;
  private String pendingSend;
  private long pendingSince;

  private boolean playing = true;
  private double playhead = 0.0;
  private double holdRemaining = 0.0;
  private long lastTickNanos = 0L;

  static double fieldLength = 16.541;
  static double fieldWidth = 8.069;

  public AutoTab(Consumer<String> sendSelection) {
    super(new BorderLayout(Theme.GAP, 0));
    this.sendSelection = sendSelection;
    setBackground(Theme.BG);
    setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));

    add(buildRoutinesCard(), BorderLayout.WEST);
    add(buildPreviewCard(), BorderLayout.CENTER);
    add(buildStepsCard(), BorderLayout.EAST);

    field.setShowLive(false);
    timeline.onSeek = this::seek;
    stepList.onSelect = index -> {
      AutoRoutine routine = previewed();
      if (routine != null && index >= 0 && index < routine.steps.size()) {
        seek(routine.steps.get(index).start + 1e-6);
      }
    };

    new Timer(33, e -> tick()).start();
    refreshPreview();
  }

  // ---- Layout -----------------------------------------------------------------------------------

  private Card buildRoutinesCard() {
    Card card = new Card("Routines", "From this laptop · connect to send");
    card.setPreferredSize(new Dimension(380, 10));
    card.addHeaderRight(sourcePill);
    routinesCard = card;

    routineList.setLayout(new BoxLayout(routineList, BoxLayout.Y_AXIS));
    routineList.setOpaque(false);
    card.content(scroll(routineList));

    JPanel selection = new JPanel(new BorderLayout(0, 10));
    selection.setOpaque(false);
    selection.setBorder(BorderFactory.createCompoundBorder(
        BorderFactory.createMatteBorder(1, 0, 0, 0, Theme.BORDER),
        BorderFactory.createEmptyBorder(14, 0, 0, 0)));

    JPanel top = new JPanel(new BorderLayout(8, 0));
    top.setOpaque(false);
    top.add(Ui.caption("Robot will run"), BorderLayout.WEST);
    top.add(activePill, BorderLayout.EAST);

    JPanel text = new JPanel(new BorderLayout(0, 6));
    text.setOpaque(false);
    text.add(top, BorderLayout.NORTH);
    text.add(activeName, BorderLayout.CENTER);

    sendButton.addActionListener(e -> send());
    selection.add(text, BorderLayout.CENTER);
    selection.add(sendButton, BorderLayout.SOUTH);
    card.add(selection, BorderLayout.SOUTH);
    return card;
  }

  private Card buildPreviewCard() {
    allianceToggle.onChange = index -> {
      allianceOverrideRed = index == 1;
      refreshPreview();
    };
    previewCard.addHeaderRight(allianceToggle);

    JPanel controls = new JPanel(new BorderLayout(10, 0));
    controls.setOpaque(false);

    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 0));
    buttons.setOpaque(false);
    playButton.addActionListener(e -> {
      playing = !playing;
      if (playing && holdRemaining <= 0 && previewed() != null
          && playhead >= previewed().totalSeconds) {
        playhead = 0.0;
      }
      updatePlayButton();
    });
    restartButton.addActionListener(e -> {
      playhead = 0.0;
      holdRemaining = 0.0;
      playing = true;
      updatePlayButton();
    });
    buttons.add(playButton);
    buttons.add(restartButton);

    timeLabel.setHorizontalAlignment(SwingConstants.RIGHT);
    timeLabel.setPreferredSize(new Dimension(104, 24));

    controls.add(buttons, BorderLayout.WEST);
    controls.add(timeline, BorderLayout.CENTER);
    controls.add(timeLabel, BorderLayout.EAST);

    JPanel south = new JPanel(new BorderLayout(0, 8));
    south.setOpaque(false);
    south.add(controls, BorderLayout.CENTER);
    south.add(new Legend(), BorderLayout.SOUTH);

    JPanel body = new JPanel(new BorderLayout(0, 12));
    body.setOpaque(false);
    body.add(field, BorderLayout.CENTER);
    body.add(south, BorderLayout.SOUTH);
    previewCard.content(body);
    return previewCard;
  }

  private Card buildStepsCard() {
    Card card = new Card("Steps", "In order, with timing");
    card.setPreferredSize(new Dimension(282, 10));
    card.content(scroll(stepList));
    return card;
  }

  private static JScrollPane scroll(JComponent view) {
    JScrollPane scroll = new JScrollPane(view);
    scroll.setBorder(null);
    scroll.setOpaque(false);
    scroll.getViewport().setOpaque(false);
    scroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    scroll.getVerticalScrollBar().setUnitIncrement(16);
    return scroll;
  }

  // ---- Inputs -----------------------------------------------------------------------------------

  /** Field size and tag layout, once the robot has published them. */
  public void setFieldLayout(double length, double width, double[] tags) {
    fieldLength = length;
    fieldWidth = width;
    field.setFieldSize(length, width);
    field.setTagLayout(tags);
    field.repaint();
    cards.values().forEach(Component::repaint);
  }

  /**
   * Called every dashboard refresh.
   *
   * @param catalogJson the robot's preview catalog, or empty if it has not sent one
   * @param options the chooser's option labels
   * @param activeOption what the robot reports it will run
   * @param alliance the robot's alliance: "Red", "Blue", or anything else for unknown
   */
  public void update(boolean isConnected, String catalogJson, String[] options, String activeOption,
      String alliance) {
    connected = isConnected;

    // The robot is the source of truth once it has sent anything. Until then (or with no robot
    // at all) the routines come from this laptop's copy of the project.
    boolean fromRobot = isConnected && (!catalogJson.isBlank() || options.length > 0);
    String catalog = fromRobot ? catalogJson : offlineCatalog;
    List<String> optionList = fromRobot ? List.of(options) : List.of();
    if (fromRobot == showingOffline || !catalog.equals(lastCatalog)
        || !optionList.equals(lastOptions)) {
      showingOffline = !fromRobot;
      lastCatalog = catalog;
      lastOptions = optionList;
      rebuildRoutines();
      updateSource();
    }

    if (!alliance.equalsIgnoreCase(robotAlliance)) {
      robotAlliance = alliance;
      allianceOverrideRed = null;
      refreshPreview();
    }

    active = activeOption == null ? "" : activeOption;
    if (pendingSend != null && pendingSend.equals(active)) {
      pendingSend = null;
    }
    for (RoutineCard card : cards.values()) {
      card.setOnRobot(connected && card.routine.label.equals(active));
    }
    updateSelectionPanel();
  }

  /**
   * Routines built from this laptop's project files, for when no robot is connected.
   *
   * @param catalogJson the catalog, or empty if there is none
   * @param problem why there is none, or null
   */
  public void setOfflineCatalog(String catalogJson, String problem) {
    offlineCatalog = catalogJson == null ? "" : catalogJson;
    offlineProblem = problem;
    if (showingOffline) {
      lastCatalog = null;   // rebuilt on the next refresh
    }
  }

  /** Says where the list came from, and whether anything can be sent. */
  private void updateSource() {
    if (showingOffline) {
      sourcePill.set("FROM LAPTOP", Theme.TEXT_DIM);
      sourcePill.setToolTipText("Built from this laptop's copy of the project. Connect to the "
          + "robot to see exactly what is deployed, and to send a routine.");
      routinesCard.setSubtitle("From this laptop · connect to send");
    } else {
      sourcePill.set("FROM ROBOT", Theme.GOOD);
      sourcePill.setToolTipText("What is deployed on the robot right now.");
      routinesCard.setSubtitle("Click to preview · double-click to send");
    }
  }

  /** The routine the robot will run, if it sent a preview for it. */
  public AutoRoutine activeRoutine() {
    return routineFor(active);
  }

  public boolean isRedPreview() {
    return allianceOverrideRed != null ? allianceOverrideRed : "red".equalsIgnoreCase(robotAlliance);
  }

  private AutoRoutine routineFor(String label) {
    for (AutoRoutine routine : routines) {
      if (routine.label.equals(label)) {
        return routine;
      }
    }
    return null;
  }

  private AutoRoutine previewed() {
    return routineFor(previewLabel);
  }

  private void rebuildRoutines() {
    List<AutoRoutine> parsed = new ArrayList<>();
    catalogProblem = null;
    if (!lastCatalog.isBlank()) {
      try {
        parsed.addAll(AutoRoutine.parseCatalog(lastCatalog));
      } catch (Exception e) {
        catalogProblem = (showingOffline ? "This laptop's" : "The robot's")
            + " auto previews could not be read: " + e.getMessage();
      }
    } else if (showingOffline) {
      catalogProblem = offlineProblem;
    }
    // Anything the chooser offers that has no preview still has to be selectable.
    for (String option : lastOptions) {
      boolean known = parsed.stream().anyMatch(r -> r.label.equals(option));
      if (!known) {
        parsed.add(AutoRoutine.placeholder(option, false));
      }
    }
    routines = parsed;

    cards.clear();
    routineList.removeAll();
    for (AutoRoutine routine : routines) {
      RoutineCard card = new RoutineCard(routine);
      card.setAlignmentX(Component.LEFT_ALIGNMENT);
      cards.put(routine.label, card);
      routineList.add(card);
      routineList.add(Box.createVerticalStrut(8));
    }
    routineList.revalidate();
    routineList.repaint();

    if (routineFor(previewLabel) == null) {
      String initial = !active.isEmpty() && routineFor(active) != null ? active : null;
      if (initial == null) {
        initial = routines.stream().filter(r -> r.isDefault).map(r -> r.label).findFirst()
            .orElse(routines.isEmpty() ? null : routines.get(0).label);
      }
      preview(initial);
    } else {
      preview(previewLabel);
    }
  }

  // ---- Actions ----------------------------------------------------------------------------------

  private void preview(String label) {
    boolean changed = label == null || !label.equals(previewLabel);
    previewLabel = label;
    for (RoutineCard card : cards.values()) {
      card.setPreviewing(card.routine.label.equals(label));
    }
    if (changed) {
      playhead = 0.0;
      holdRemaining = 0.0;
      playing = true;
      updatePlayButton();
    }
    refreshPreview();
    updateSelectionPanel();
  }

  private void send() {
    if (previewLabel == null || !connected) {
      return;
    }
    sendSelection.accept(previewLabel);
    pendingSend = previewLabel;
    pendingSince = System.nanoTime();
    updateSelectionPanel();
  }

  private void seek(double seconds) {
    AutoRoutine routine = previewed();
    if (routine == null) {
      return;
    }
    playhead = Math.max(0.0, Math.min(routine.totalSeconds, seconds));
    holdRemaining = 0.0;
    playing = false;
    updatePlayButton();
    applyPlayhead();
  }

  private void updatePlayButton() {
    playButton.setIcon(playing ? Icons.pause(14, Theme.TEXT) : Icons.play(14, Theme.TEXT));
    playButton.setToolTipText(playing ? "Pause preview" : "Play preview");
  }

  // ---- Refresh ----------------------------------------------------------------------------------

  private void refreshPreview() {
    AutoRoutine routine = previewed();
    boolean red = isRedPreview();

    allianceToggle.setSelected(red ? 1 : 0);
    boolean known = "red".equalsIgnoreCase(robotAlliance) || "blue".equalsIgnoreCase(robotAlliance);
    String side = red ? "red" : "blue";
    String allianceNote;
    if (allianceOverrideRed != null) {
      allianceNote = "previewing the " + side + " side";
    } else if (known) {
      allianceNote = side + " alliance, from the robot";
    } else {
      allianceNote = "alliance unknown, showing blue";
    }

    if (routine == null) {
      previewCard.setTitle("Autonomous");
      previewCard.setSubtitle(connected ? "Waiting for routines" : "Not connected");
      field.setRoutine(null, red, false);
      field.setMessage(catalogProblem != null ? catalogProblem
          : connected ? "Waiting for the robot to send its autonomous routines…"
              : "No autonomous routines to show.");
    } else {
      previewCard.setTitle(routine.label);
      previewCard.setSubtitle(routine.summary() + "  ·  " + allianceNote);
      field.setRoutine(routine, red, false);
      if (routine.error != null) {
        field.setMessage("No preview: " + routine.error);
      } else if (!routine.drives()) {
        field.setMessage(routine.note != null ? routine.note : "This routine does not drive.");
      } else {
        field.setMessage(null);
      }
    }
    // Autonomous stays in its own half, so show that half (and a little past the centre line).
    double half = fieldLength / 2.0;
    field.setViewX(red ? half - 0.9 : 0.0, red ? fieldLength : half + 0.9);

    timeline.setRoutine(routine);
    stepList.setRoutine(routine);
    cards.values().forEach(card -> card.setRed(red));
    applyPlayhead();
  }

  private void updateSelectionPanel() {
    boolean waiting = pendingSend != null
        && System.nanoTime() - pendingSince < CONFIRM_TIMEOUT_NANOS;

    if (!connected) {
      activeName.setText("Robot not connected");
      activeName.setForeground(Theme.TEXT_DIM);
      activePill.set("OFFLINE", Theme.TEXT_FAINT);
    } else if (active.isEmpty()) {
      activeName.setText("Nothing selected");
      activeName.setForeground(Theme.WARN);
      activePill.set("NONE", Theme.WARN);
    } else {
      activeName.setText(active);
      activeName.setForeground(Theme.TEXT);
      if (pendingSend != null) {
        activePill.set(waiting ? "SENDING" : "NOT CONFIRMED", waiting ? Theme.ACCENT : Theme.BAD);
      } else {
        activePill.set("CONFIRMED", Theme.GOOD);
      }
    }

    boolean isActive = previewLabel != null && previewLabel.equals(active);
    if (!connected) {
      sendButton.setEnabled(false);
      sendButton.setText("Send to robot");
      sendButton.setIcon(Icons.send(14, Theme.TEXT_FAINT));
    } else if (previewLabel == null) {
      sendButton.setEnabled(false);
      sendButton.setText("Pick a routine");
      sendButton.setIcon(null);
    } else if (isActive) {
      sendButton.setEnabled(false);
      sendButton.setText("Selected on robot");
      sendButton.setIcon(Icons.check(14, Theme.TEXT_FAINT));
    } else {
      sendButton.setEnabled(!waiting);
      sendButton.setText("Send to robot");
      sendButton.setIcon(Icons.send(14, Theme.TEXT_ON_ACCENT));
    }
  }

  private void tick() {
    long now = System.nanoTime();
    double dt = lastTickNanos == 0L ? 0.0 : Math.min(0.1, (now - lastTickNanos) / 1e9);
    lastTickNanos = now;
    if (pendingSend != null) {
      updateSelectionPanel();
    }
    if (!isShowing()) {
      return;
    }
    AutoRoutine routine = previewed();
    if (routine == null || routine.totalSeconds <= 0.0) {
      return;
    }
    if (playing) {
      if (holdRemaining > 0.0) {
        holdRemaining -= dt;
        if (holdRemaining <= 0.0) {
          playhead = 0.0;
        }
      } else {
        playhead += dt;
        if (playhead >= routine.totalSeconds) {
          playhead = routine.totalSeconds;
          holdRemaining = END_HOLD_SECONDS;
        }
      }
      applyPlayhead();
    }
  }

  private void applyPlayhead() {
    AutoRoutine routine = previewed();
    if (routine == null || routine.steps.isEmpty()) {
      field.setPlayhead(Double.NaN);
      timeline.setPlayhead(0.0);
      stepList.setActive(-1, 0.0);
      timeLabel.setText("");
    } else {
      field.setPlayhead(playhead);
      timeline.setPlayhead(playhead);
      stepList.setActive(routine.stepIndexAt(playhead), playhead);
      timeLabel.setText(String.format(Locale.ROOT, "%.1f / %.1f s", playhead, routine.totalSeconds));
    }
    field.repaint();
  }

  // ---- Routine card -----------------------------------------------------------------------------

  /** One routine in the list: a thumbnail of its route, its name, and what it does in brief. */
  private final class RoutineCard extends JComponent {

    private static final int HEIGHT = 76;

    final AutoRoutine routine;
    private boolean hover;
    private boolean previewing;
    private boolean onRobot;
    private boolean red;

    RoutineCard(AutoRoutine routine) {
      this.routine = routine;
      setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
      addMouseListener(new MouseAdapter() {
        @Override
        public void mouseEntered(MouseEvent e) {
          hover = true;
          repaint();
        }

        @Override
        public void mouseExited(MouseEvent e) {
          hover = false;
          repaint();
        }

        @Override
        public void mousePressed(MouseEvent e) {
          preview(routine.label);
          if (e.getClickCount() >= 2) {
            send();
          }
        }
      });
    }

    void setPreviewing(boolean on) {
      if (previewing != on) {
        previewing = on;
        repaint();
      }
    }

    void setOnRobot(boolean on) {
      if (onRobot != on) {
        onRobot = on;
        repaint();
      }
    }

    void setRed(boolean on) {
      if (red != on) {
        red = on;
        repaint();
      }
    }

    @Override
    public Dimension getPreferredSize() {
      return new Dimension(260, HEIGHT);
    }

    @Override
    public Dimension getMaximumSize() {
      return new Dimension(Integer.MAX_VALUE, HEIGHT);
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      int w = getWidth();
      int h = getHeight();
      RoundRectangle2D shape = new RoundRectangle2D.Double(0.5, 0.5, w - 1.0, h - 1.0, 12, 12);

      g2.setColor(previewing ? Theme.alpha(Theme.ACCENT, 22)
          : hover ? Theme.SURFACE_HOVER : Theme.SURFACE_RAISED);
      g2.fill(shape);
      g2.setStroke(new BasicStroke(previewing ? 1.6f : 1f));
      g2.setColor(previewing ? Theme.alpha(Theme.ACCENT, 200) : Theme.BORDER);
      g2.draw(shape);

      // Thumbnail.
      int mapW = 76;
      int mapH = (int) Math.round(mapW * fieldWidth / fieldLength);
      int mapX = 12;
      int mapY = (h - mapH) / 2;
      drawThumbnail(g2, mapX, mapY, mapW, mapH);

      int textX = mapX + mapW + 14;
      int textW = w - textX - 12;
      g2.setFont(Theme.ui(Font.BOLD, 13f));
      g2.setColor(Theme.TEXT);
      g2.drawString(StatTile.ellipsize(g2.getFontMetrics(), routine.label, textW), textX, 26);

      g2.setFont(Theme.ui(Font.PLAIN, 12f));
      g2.setColor(routine.error != null ? Theme.BAD : Theme.TEXT_DIM);
      g2.drawString(StatTile.ellipsize(g2.getFontMetrics(), routine.summary(), textW), textX, 45);

      int badgeX = textX;
      if (onRobot) {
        badgeX = badge(g2, badgeX, 54, "ON ROBOT", Theme.GOOD, true);
      }
      if (routine.isDefault) {
        badge(g2, badgeX, 54, "DEFAULT", Theme.TEXT_DIM, false);
      }
      g2.dispose();
    }

    private int badge(Graphics2D g2, int x, int y, String text, Color colour, boolean dot) {
      g2.setFont(Theme.caption(9.5f));
      FontMetrics fm = g2.getFontMetrics();
      int w = fm.stringWidth(text) + (dot ? 24 : 14);
      int h = 17;
      g2.setColor(Theme.alpha(colour, 32));
      g2.fill(new RoundRectangle2D.Double(x, y, w, h, h, h));
      int tx = x + 7;
      if (dot) {
        g2.setColor(colour);
        g2.fill(new Ellipse2D.Double(x + 7, y + h / 2.0 - 3, 6, 6));
        tx += 10;
      }
      g2.setColor(Theme.mix(colour, Theme.TEXT, 0.2));
      g2.drawString(text, tx, y + (h + fm.getAscent() - fm.getDescent()) / 2);
      return x + w + 6;
    }

    private void drawThumbnail(Graphics2D g2, int x, int y, int w, int h) {
      RoundRectangle2D frame = new RoundRectangle2D.Double(x, y, w, h, 6, 6);
      g2.setColor(new Color(0x1A1F28));
      g2.fill(frame);
      double s = w / fieldLength;

      // Alliance ends and HUBs, just enough to orient the route.
      g2.setColor(Theme.alpha(Theme.ALLIANCE_BLUE, 40));
      g2.fill(new Rectangle2D.Double(x, y, 4.03 * s, h));
      g2.setColor(Theme.alpha(Theme.ALLIANCE_RED, 40));
      g2.fill(new Rectangle2D.Double(x + w - 4.03 * s, y, 4.03 * s, h));
      double hub = 1.194 * s;
      g2.setColor(Theme.alpha(Theme.ALLIANCE_BLUE, 150));
      g2.fill(new Rectangle2D.Double(x + 4.625 * s - hub / 2, y + h / 2.0 - hub / 2, hub, hub));
      g2.setColor(Theme.alpha(Theme.ALLIANCE_RED, 150));
      g2.fill(new Rectangle2D.Double(x + w - 4.625 * s - hub / 2, y + h / 2.0 - hub / 2, hub, hub));
      g2.setColor(Theme.BORDER_STRONG);
      g2.draw(frame);

      if (!routine.drives()) {
        return;
      }
      for (AutoRoutine.Step step : routine.steps) {
        if (step.kind != AutoRoutine.Kind.DRIVE || step.points == null || step.points.length < 8) {
          continue;
        }
        Path2D path = new Path2D.Double();
        for (int i = 0; i + 3 < step.points.length; i += 4) {
          double fx = red ? fieldLength - step.points[i + 1] : step.points[i + 1];
          double fy = red ? fieldWidth - step.points[i + 2] : step.points[i + 2];
          double px = x + fx * s;
          double py = y + (fieldWidth - fy) * s;
          if (i == 0) {
            path.moveTo(px, py);
          } else {
            path.lineTo(px, py);
          }
        }
        g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g2.setColor(FieldView.colourOf(step));
        g2.draw(path);
      }
      for (AutoRoutine.Step step : routine.steps) {
        if (step.kind == AutoRoutine.Kind.SHOT && step.located()) {
          double fx = red ? fieldLength - step.x : step.x;
          double fy = red ? fieldWidth - step.y : step.y;
          g2.setColor(Theme.ROUTE_SHOT);
          g2.fill(new Ellipse2D.Double(x + fx * s - 2.5, y + (fieldWidth - fy) * s - 2.5, 5, 5));
        }
      }
    }
  }

  // ---- Timeline ---------------------------------------------------------------------------------

  /** The routine as coloured segments over time, with a draggable playhead. */
  private static final class Timeline extends JComponent {

    private AutoRoutine routine;
    private double playhead;
    DoubleConsumer onSeek = t -> { };

    Timeline() {
      setPreferredSize(new Dimension(300, 40));
      setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
      MouseAdapter scrub = new MouseAdapter() {
        @Override
        public void mousePressed(MouseEvent e) {
          seekTo(e.getX());
        }

        @Override
        public void mouseDragged(MouseEvent e) {
          seekTo(e.getX());
        }
      };
      addMouseListener(scrub);
      addMouseMotionListener(scrub);
    }

    void setRoutine(AutoRoutine newRoutine) {
      routine = newRoutine;
      repaint();
    }

    void setPlayhead(double t) {
      if (t != playhead) {
        playhead = t;
        repaint();
      }
    }

    private Rectangle track() {
      return new Rectangle(8, 10, Math.max(1, getWidth() - 16), 14);
    }

    private void seekTo(int x) {
      if (routine == null || routine.totalSeconds <= 0) {
        return;
      }
      Rectangle r = track();
      double u = Math.max(0.0, Math.min(1.0, (x - r.x) / (double) r.width));
      onSeek.accept(u * routine.totalSeconds);
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      Rectangle r = track();
      g2.setColor(Theme.SURFACE_RAISED);
      g2.fill(new RoundRectangle2D.Double(r.x, r.y, r.width, r.height, r.height, r.height));

      if (routine == null || routine.totalSeconds <= 0) {
        if (routine != null) {
          g2.setFont(Theme.ui(Font.PLAIN, 11f));
          g2.setColor(Theme.TEXT_FAINT);
          g2.drawString("Nothing happens during autonomous", r.x + 10, r.y + 11);
        }
        g2.dispose();
        return;
      }

      double total = routine.totalSeconds;
      java.awt.Shape clip = g2.getClip();
      g2.clip(new RoundRectangle2D.Double(r.x, r.y, r.width, r.height, r.height, r.height));
      for (AutoRoutine.Step step : routine.steps) {
        if (step.seconds() <= 0) {
          continue;
        }
        double x0 = r.x + r.width * step.start / total;
        double x1 = r.x + r.width * step.end / total;
        Color colour = FieldView.colourOf(step);
        boolean past = playhead >= step.end;
        boolean now = playhead >= step.start && playhead < step.end;
        g2.setColor(Theme.alpha(colour, now ? 255 : past ? 170 : 90));
        g2.fill(new Rectangle2D.Double(x0 + 1, r.y, Math.max(1, x1 - x0 - 2), r.height));
      }
      g2.setClip(clip);

      // One-second ticks under the track.
      g2.setColor(Theme.alpha(Theme.TEXT_FAINT, 160));
      g2.setStroke(new BasicStroke(1f));
      for (int s = 0; s <= total; s++) {
        double x = r.x + r.width * s / total;
        int len = s % 5 == 0 ? 6 : 3;
        g2.draw(new Line2D.Double(x, r.y + r.height + 3, x, r.y + r.height + 3 + len));
      }

      // Playhead.
      double px = r.x + r.width * Math.min(1.0, playhead / total);
      g2.setColor(Theme.TEXT);
      g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g2.draw(new Line2D.Double(px, r.y - 4, px, r.y + r.height + 4));
      g2.fill(new Ellipse2D.Double(px - 5, r.y - 9, 10, 10));
      g2.dispose();
    }
  }

  // ---- Step list --------------------------------------------------------------------------------

  /** Numbered steps matching the badges on the field. The running one is highlighted. */
  private static final class StepList extends JComponent implements Scrollable {

    private static final int ROW = 60;

    private AutoRoutine routine;
    private int active = -1;
    private double playhead;
    IntConsumer onSelect = i -> { };

    StepList() {
      setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
      addMouseListener(new MouseAdapter() {
        @Override
        public void mousePressed(MouseEvent e) {
          onSelect.accept(e.getY() / ROW);
        }
      });
    }

    void setRoutine(AutoRoutine newRoutine) {
      routine = newRoutine;
      revalidate();
      repaint();
    }

    void setActive(int index, double t) {
      if (index != active || t != playhead) {
        boolean scroll = index != active;
        active = index;
        playhead = t;
        if (scroll && index >= 0) {
          scrollRectToVisible(new Rectangle(0, index * ROW, 10, ROW));
        }
        repaint();
      }
    }

    @Override
    public Dimension getPreferredSize() {
      int rows = routine == null ? 1 : Math.max(1, routine.steps.size());
      return new Dimension(240, rows * ROW);
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      int w = getWidth();

      if (routine == null || routine.steps.isEmpty()) {
        g2.setFont(Theme.ui(Font.PLAIN, 12f));
        g2.setColor(Theme.TEXT_DIM);
        g2.drawString(routine == null ? "No routine selected" : "No steps: the robot stays still",
            4, 22);
        g2.dispose();
        return;
      }

      for (int i = 0; i < routine.steps.size(); i++) {
        AutoRoutine.Step step = routine.steps.get(i);
        int y = i * ROW;
        boolean isActive = i == active && playhead < step.end;
        boolean done = playhead >= step.end && step.seconds() > 0 || playhead > step.end;

        if (isActive) {
          g2.setColor(Theme.SURFACE_HOVER);
          g2.fill(new RoundRectangle2D.Double(0, y + 2, w, ROW - 4, 10, 10));
          g2.setColor(FieldView.colourOf(step));
          g2.fill(new RoundRectangle2D.Double(0, y + 12, 3, ROW - 24, 3, 3));
        }

        // Connector between badges, so the list reads as a sequence.
        if (i < routine.steps.size() - 1) {
          g2.setColor(Theme.BORDER_STRONG);
          g2.setStroke(new BasicStroke(1.5f));
          g2.draw(new Line2D.Double(26, y + ROW / 2.0 + 13, 26, y + ROW + ROW / 2.0 - 13));
        }

        Color colour = FieldView.colourOf(step);
        double cx = 26;
        double cy = y + ROW / 2.0;
        g2.setColor(done ? Theme.alpha(colour, 120) : colour);
        g2.fill(new Ellipse2D.Double(cx - 11, cy - 11, 22, 22));
        g2.setFont(Theme.numeric(Font.BOLD, 12f));
        FontMetrics nfm = g2.getFontMetrics();
        String number = String.valueOf(i + 1);
        g2.setColor(Theme.TEXT_ON_ACCENT);
        g2.drawString(number, (float) (cx - nfm.stringWidth(number) / 2.0),
            (float) (cy + (nfm.getAscent() - nfm.getDescent()) / 2.0));

        int textX = 48;
        g2.setFont(Theme.numeric(Font.PLAIN, 12f));
        String when = String.format(Locale.ROOT, "%.1f s", step.start);
        int whenW = g2.getFontMetrics().stringWidth(when);
        g2.setColor(Theme.TEXT_FAINT);
        g2.drawString(when, w - whenW - 8, y + 25);

        int textW = w - textX - whenW - 16;
        g2.setFont(Theme.ui(Font.BOLD, 13f));
        g2.setColor(isActive ? Theme.TEXT : done ? Theme.TEXT_DIM : Theme.TEXT);
        g2.drawString(StatTile.ellipsize(g2.getFontMetrics(), step.title, textW), textX, y + 25);

        g2.setFont(Theme.ui(Font.PLAIN, 11.5f));
        g2.setColor(Theme.TEXT_DIM);
        g2.drawString(StatTile.ellipsize(g2.getFontMetrics(), kindLabel(step) + "  ·  "
            + step.detail, w - textX - 8), textX, y + 43);
      }
      g2.dispose();
    }

    private static String kindLabel(AutoRoutine.Step step) {
      switch (step.kind) {
        case DRIVE:
          return step.intake ? "Drive + intake" : "Drive";
        case SHOT:
          return "Shot";
        case WAIT:
          return "Wait";
        default:
          return "Action";
      }
    }

    @Override
    public Dimension getPreferredScrollableViewportSize() {
      return getPreferredSize();
    }

    @Override
    public int getScrollableUnitIncrement(Rectangle visible, int orientation, int direction) {
      return ROW / 2;
    }

    @Override
    public int getScrollableBlockIncrement(Rectangle visible, int orientation, int direction) {
      return visible.height;
    }

    @Override
    public boolean getScrollableTracksViewportWidth() {
      return true;
    }

    @Override
    public boolean getScrollableTracksViewportHeight() {
      return false;
    }
  }

  // ---- Small parts ------------------------------------------------------------------------------

  /** What the route colours mean. */
  private static final class Legend extends JComponent {

    private static final Object[][] ITEMS = {
      {"Drive", Theme.ROUTE_DRIVE},
      {"Drive + intake", Theme.ROUTE_INTAKE},
      {"Shot", Theme.ROUTE_SHOT},
      {"Aim at HUB", Theme.ROUTE_SHOT},
      {"Start", Theme.TEXT}
    };

    Legend() {
      setPreferredSize(new Dimension(300, 18));
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      g2.setFont(Theme.ui(Font.PLAIN, 12f));
      FontMetrics fm = g2.getFontMetrics();
      int x = 4;
      int cy = getHeight() / 2;
      for (Object[] item : ITEMS) {
        String text = (String) item[0];
        Color colour = (Color) item[1];
        if ("Aim at HUB".equals(text)) {
          g2.setColor(colour);
          g2.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f,
              new float[] {2f, 4f}, 0f));
          g2.draw(new Line2D.Double(x, cy, x + 14, cy));
        } else if ("Start".equals(text)) {
          g2.setColor(colour);
          g2.setStroke(new BasicStroke(1.4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f,
              new float[] {3f, 2f}, 0f));
          g2.draw(new RoundRectangle2D.Double(x, cy - 5, 10, 10, 3, 3));
        } else {
          g2.setColor(colour);
          g2.setStroke(new BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
          g2.draw(new Line2D.Double(x, cy, x + 14, cy));
        }
        x += 20;
        g2.setColor(Theme.TEXT_DIM);
        g2.drawString(text, x, cy + (fm.getAscent() - fm.getDescent()) / 2);
        x += fm.stringWidth(text) + 18;
      }
      g2.dispose();
    }
  }

  /** A two-way switch. Here, which alliance's side to draw the route on. */
  private static final class Segmented extends JComponent {

    private final String[] options;
    private int selected = 0;
    IntConsumer onChange = i -> { };

    Segmented(String... options) {
      this.options = options;
      setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
      setPreferredSize(new Dimension(options.length * 64, 30));
      addMouseListener(new MouseAdapter() {
        @Override
        public void mousePressed(MouseEvent e) {
          int index = Math.min(options.length - 1, e.getX() * options.length / Math.max(1, getWidth()));
          if (index != selected) {
            selected = index;
            repaint();
            onChange.accept(index);
          }
        }
      });
    }

    void setSelected(int index) {
      if (index != selected) {
        selected = index;
        repaint();
      }
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      int w = getWidth();
      int h = 28;
      int y = (getHeight() - h) / 2;
      g2.setColor(Theme.SURFACE_RAISED);
      g2.fill(new RoundRectangle2D.Double(0, y, w, h, h, h));
      g2.setColor(Theme.BORDER_STRONG);
      g2.draw(new RoundRectangle2D.Double(0.5, y + 0.5, w - 1, h - 1, h, h));

      double segment = w / (double) options.length;
      g2.setFont(Theme.ui(Font.BOLD, 12f));
      FontMetrics fm = g2.getFontMetrics();
      for (int i = 0; i < options.length; i++) {
        double x = i * segment;
        if (i == selected) {
          Color fill = i == 1 ? Theme.ALLIANCE_RED : Theme.ALLIANCE_BLUE;
          g2.setColor(fill);
          g2.fill(new RoundRectangle2D.Double(x + 3, y + 3, segment - 6, h - 6, h - 6, h - 6));
        }
        g2.setColor(i == selected ? Color.WHITE : Theme.TEXT_DIM);
        g2.drawString(options[i], (float) (x + (segment - fm.stringWidth(options[i])) / 2),
            y + (h + fm.getAscent() - fm.getDescent()) / 2f);
      }
      g2.dispose();
    }
  }
}
