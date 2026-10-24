package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.Font;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;

import edu.wpi.first.networktables.NetworkTable;

/**
 * Two-step arming for CYBERPSYCHO, and a way to back out of it.
 *
 * <p>Arming is not the mode. It is a decision that AUTO will START in the mode, taken before the
 * match while there is still time to change your mind, and either confirmation can be withdrawn
 * right up until AUTO begins. After that the robot's own latch closes and nothing here matters.
 *
 * <p>Two steps rather than one because a single button that throws the robot into something it
 * cannot leave is a button somebody eventually leans on. Two deliberate presses, with the second
 * spelling out what it does, is the cheapest protection that still takes five seconds to use.
 *
 * <p>The TEST toggle drives the dashboard's own distortion with no robot attached, so the visual
 * side can be worked on at a desk. It deliberately does NOT write the robot's arming keys: a
 * dashboard that can arm a mode by pretending is worse than one that cannot preview it.
 */
class PsychoArmPanel extends JPanel {

  private final NetworkTable dash;
  private final JLabel state = new JLabel();
  private final JButton step = new JButton();
  private final JButton back = new JButton("Back out");
  private final JButton test = new JButton("Test UI");

  private int confirmations;
  private boolean testing;
  private boolean robotArmed;
  private boolean robotActive;

  PsychoArmPanel(NetworkTable dash) {
    this.dash = dash;
    setLayout(new BorderLayout(0, 8));
    setOpaque(false);

    state.setFont(Theme.ui(Font.PLAIN, 12.5f));
    state.setForeground(Theme.TEXT_DIM);

    step.addActionListener(e -> advance());
    back.addActionListener(e -> reset());
    test.addActionListener(e -> {
      testing = !testing;
      refreshControls();
    });

    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 0));
    buttons.setOpaque(false);
    buttons.add(step);
    buttons.add(back);
    buttons.add(test);

    add(state, BorderLayout.NORTH);
    add(buttons, BorderLayout.CENTER);
    setBorder(BorderFactory.createEmptyBorder(4, 0, 0, 0));
    refreshControls();
  }

  /** True when the dashboard should distort: the robot is in the mode, or we are previewing it. */
  boolean shouldDistort() {
    return robotActive || testing;
  }

  boolean isTesting() {
    return testing;
  }

  /** Called each refresh with what the robot says about itself. */
  void update(boolean armed, boolean active) {
    this.robotArmed = armed;
    this.robotActive = active;
    refreshControls();
  }

  private void advance() {
    if (robotActive) {
      return;
    }
    confirmations = Math.min(CyberPsychoKeys.CONFIRMATIONS_REQUIRED, confirmations + 1);
    push();
  }

  private void reset() {
    confirmations = 0;
    testing = false;
    push();
  }

  private void push() {
    dash.getEntry(CyberPsychoKeys.CONFIRMATIONS).setDouble(confirmations);
    refreshControls();
  }

  private void refreshControls() {
    if (robotActive) {
      state.setText("CYBERPSYCHO is live. There is no way out but a reboot.");
      step.setEnabled(false);
      back.setEnabled(false);
      step.setText("Engaged");
    } else if (robotArmed || confirmations >= CyberPsychoKeys.CONFIRMATIONS_REQUIRED) {
      state.setText("ARMED. AUTO will start in CYBERPSYCHO. Back out any time before it does.");
      step.setEnabled(false);
      back.setEnabled(true);
      step.setText("Armed");
    } else if (confirmations == 1) {
      state.setText("Confirm again: AUTO will start in CYBERPSYCHO and cannot be left.");
      step.setEnabled(true);
      back.setEnabled(true);
      step.setText("Yes, I am sure");
    } else {
      state.setText("Start AUTO in CYBERPSYCHO? Takes two confirmations.");
      step.setEnabled(true);
      back.setEnabled(false);
      step.setText("Arm CYBERPSYCHO");
    }
    test.setText(testing ? "Stop UI test" : "Test UI");
    repaint();
  }
}
