package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import edu.wpi.first.networktables.NetworkTable;

/**
 * Edits one control binding at a time, live.
 *
 * <p>Pick an action, pick where it lives, press Apply. The change is written to NetworkTables and
 * the robot re-points that binding immediately — no redeploy, which matters because a control
 * change you cannot try until the next build is a control change nobody makes.
 *
 * <p>Actions are searchable because there are nearly twenty of them and scrolling a list to find
 * the one you want is slower than typing three letters of it.
 */
public class BindingEditor extends JPanel {

  private static final Color BG = new Color(0x13, 0x15, 0x19);
  private static final Color PANEL = new Color(0x1C, 0x1F, 0x26);
  private static final Color TEXT = new Color(0xD8, 0xDC, 0xE2);
  private static final Color DIM = new Color(0x8A, 0x90, 0x98);
  private static final Color ACCENT = new Color(0x4C, 0xC2, 0xFF);
  private static final Color GOOD = new Color(0x3D, 0xDC, 0x84);

  private final NetworkTable controls;

  private final JTextField search = new JTextField();
  private final DefaultListModel<String> actionModel = new DefaultListModel<>();
  private final JList<String> actionList = new JList<>(actionModel);

  private final JComboBox<String> roleBox = new JComboBox<>(new String[] {"DRIVER", "OPERATOR"});
  private final JComboBox<String> inputBox = new JComboBox<>();
  private final JComboBox<String> modifierBox = new JComboBox<>();
  private final JLabel statusLabel = new JLabel(" ");

  private List<String> allActions = new ArrayList<>();
  private Map<String, String> currentMap = new LinkedHashMap<>();

  public BindingEditor(NetworkTable controls) {
    this.controls = controls;
    setLayout(new BorderLayout(10, 10));
    setBackground(BG);

    add(buildActionPicker(), BorderLayout.WEST);
    add(buildAssignment(), BorderLayout.CENTER);
  }

  private JPanel buildActionPicker() {
    JPanel panel = new JPanel(new BorderLayout(0, 6));
    panel.setBackground(BG);
    panel.setPreferredSize(new Dimension(230, 240));

    JLabel caption = new JLabel("ACTION");
    caption.setForeground(DIM);
    caption.setFont(new Font("Segoe UI", Font.BOLD, 10));

    search.setToolTipText("Type to filter actions");
    search.getDocument().addDocumentListener(new DocumentListener() {
      @Override public void insertUpdate(DocumentEvent e) {
        applyFilter();
      }

      @Override public void removeUpdate(DocumentEvent e) {
        applyFilter();
      }

      @Override public void changedUpdate(DocumentEvent e) {
        applyFilter();
      }
    });

    actionList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
    actionList.setBackground(PANEL);
    actionList.setForeground(TEXT);
    actionList.addListSelectionListener(e -> {
      if (!e.getValueIsAdjusting()) {
        showCurrentBinding();
      }
    });

    JPanel top = new JPanel(new BorderLayout(0, 4));
    top.setBackground(BG);
    top.add(caption, BorderLayout.NORTH);
    top.add(search, BorderLayout.SOUTH);

    panel.add(top, BorderLayout.NORTH);
    panel.add(new JScrollPane(actionList), BorderLayout.CENTER);
    return panel;
  }

  private JPanel buildAssignment() {
    JPanel panel = new JPanel();
    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
    panel.setBackground(BG);

    JLabel caption = new JLabel("ASSIGN TO");
    caption.setForeground(DIM);
    caption.setFont(new Font("Segoe UI", Font.BOLD, 10));
    caption.setAlignmentX(LEFT_ALIGNMENT);
    panel.add(caption);
    panel.add(Box.createVerticalStrut(6));

    panel.add(row("Controller", roleBox));
    panel.add(row("Button", inputBox));
    panel.add(row("Hold with", modifierBox));

    JLabel comboNote = new JLabel(
        "<html><body style='width:300px'>\"Hold with\" combines two buttons into one binding. "
            + "Set it to NONE for a plain single-button press.</body></html>");
    comboNote.setForeground(DIM);
    comboNote.setFont(new Font("Segoe UI", Font.PLAIN, 10));
    comboNote.setAlignmentX(LEFT_ALIGNMENT);
    panel.add(Box.createVerticalStrut(4));
    panel.add(comboNote);

    JButton apply = new JButton("Apply binding");
    apply.setAlignmentX(LEFT_ALIGNMENT);
    apply.addActionListener(e -> applyBinding());
    panel.add(Box.createVerticalStrut(10));
    panel.add(apply);

    statusLabel.setForeground(GOOD);
    statusLabel.setFont(new Font("Segoe UI", Font.PLAIN, 11));
    statusLabel.setAlignmentX(LEFT_ALIGNMENT);
    panel.add(Box.createVerticalStrut(8));
    panel.add(statusLabel);

    panel.add(Box.createVerticalGlue());
    return panel;
  }

  private JPanel row(String caption, java.awt.Component field) {
    JPanel row = new JPanel(new GridLayout(1, 2, 8, 0));
    row.setBackground(BG);
    row.setAlignmentX(LEFT_ALIGNMENT);
    row.setMaximumSize(new Dimension(360, 28));
    JLabel label = new JLabel(caption);
    label.setForeground(TEXT);
    row.add(label);
    row.add(field);
    return row;
  }

  private void applyFilter() {
    String needle = search.getText().trim().toUpperCase();
    String selected = actionList.getSelectedValue();
    actionModel.clear();
    for (String action : allActions) {
      if (needle.isEmpty() || action.contains(needle)) {
        actionModel.addElement(action);
      }
    }
    if (selected != null && actionModel.contains(selected)) {
      actionList.setSelectedValue(selected, true);
    }
  }

  /** Fills the dropdowns to reflect whatever the selected action is currently bound to. */
  private void showCurrentBinding() {
    String action = actionList.getSelectedValue();
    if (action == null) {
      return;
    }
    String binding = currentMap.get(action);
    if (binding == null || binding.isBlank()) {
      statusLabel.setText(action + " is unbound");
      return;
    }

    // Format is "ROLE MODIFIER+INPUT" or "ROLE INPUT".
    String[] parts = binding.trim().split("\\s+");
    if (parts.length >= 1) {
      roleBox.setSelectedItem(parts[0]);
    }
    if (parts.length >= 2) {
      String spec = parts[1];
      if (spec.contains("+")) {
        String[] combo = spec.split(java.util.regex.Pattern.quote("+"), 2);
        modifierBox.setSelectedItem(combo[0]);
        inputBox.setSelectedItem(combo[1]);
      } else {
        modifierBox.setSelectedItem("NONE");
        inputBox.setSelectedItem(spec);
      }
    }
    statusLabel.setText(action + "  ->  " + binding);
  }

  private void applyBinding() {
    String action = actionList.getSelectedValue();
    if (action == null) {
      statusLabel.setText("Pick an action first");
      return;
    }
    String role = String.valueOf(roleBox.getSelectedItem());
    String input = String.valueOf(inputBox.getSelectedItem());
    String modifier = String.valueOf(modifierBox.getSelectedItem());

    String spec = "NONE".equals(modifier) ? input : modifier + "+" + input;
    controls.getEntry("RebindRequest").setString(action + "=" + role + " " + spec);
    statusLabel.setText("Applied: " + action + " -> " + role + " " + spec);
  }

  /** Called each refresh with the robot's current action list, input list and active map. */
  public void update(String[] actions, String[] inputs, String activeMap) {
    if (actions.length > 0 && actions.length != allActions.size()) {
      allActions = List.of(actions);
      applyFilter();
    }
    if (inputs.length > 0 && inputs.length != inputBox.getItemCount()) {
      inputBox.removeAllItems();
      modifierBox.removeAllItems();
      for (String input : inputs) {
        inputBox.addItem(input);
        modifierBox.addItem(input);
      }
      modifierBox.setSelectedItem("NONE");
    }

    currentMap = parseMap(activeMap);
  }

  /** The robot publishes the active map as {@code ACTION=ROLE INPUT} lines. */
  private static Map<String, String> parseMap(String raw) {
    Map<String, String> map = new LinkedHashMap<>();
    if (raw == null || raw.isBlank()) {
      return map;
    }
    for (String line : raw.split("\n")) {
      int eq = line.indexOf('=');
      if (eq > 0) {
        map.put(line.substring(0, eq).trim(), line.substring(eq + 1).trim());
      }
    }
    return map;
  }

  /** Input name to action, for the controller diagram, split by which controller it is on. */
  public Map<String, String> bindingsForRole(String role) {
    Map<String, String> byInput = new LinkedHashMap<>();
    currentMap.forEach((action, binding) -> {
      String[] parts = binding.trim().split("\\s+");
      if (parts.length < 2 || !parts[0].equalsIgnoreCase(role)) {
        return;
      }
      String spec = parts[1];
      // A combo is shown on its primary button; the modifier is noted in the label.
      if (spec.contains("+")) {
        String[] combo = spec.split(java.util.regex.Pattern.quote("+"), 2);
        byInput.put(combo[1], combo[0] + "+ " + action);
      } else {
        byInput.put(spec, action);
      }
    });
    return byInput;
  }
}
