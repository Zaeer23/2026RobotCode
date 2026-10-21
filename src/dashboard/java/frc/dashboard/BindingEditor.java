package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
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

  private static final Color TEXT = Theme.TEXT;
  private static final Color GOOD = Theme.GOOD;

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
    setLayout(new BorderLayout(16, 10));
    setOpaque(false);

    add(buildActionPicker(), BorderLayout.WEST);
    add(buildAssignment(), BorderLayout.CENTER);
  }

  private JPanel buildActionPicker() {
    JPanel panel = new JPanel(new BorderLayout(0, 8));
    panel.setOpaque(false);
    panel.setPreferredSize(new Dimension(260, 240));

    JLabel caption = Ui.caption("Action");

    search.setToolTipText("Type to filter actions");
    search.putClientProperty(com.formdev.flatlaf.FlatClientProperties.PLACEHOLDER_TEXT,
        "Search actions…");
    search.putClientProperty(com.formdev.flatlaf.FlatClientProperties.TEXT_FIELD_SHOW_CLEAR_BUTTON,
        true);
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
    actionList.setBackground(Theme.SURFACE_RAISED);
    actionList.setForeground(TEXT);
    actionList.setFont(Theme.ui(Font.PLAIN, 12.5f));
    actionList.addListSelectionListener(e -> {
      if (!e.getValueIsAdjusting()) {
        showCurrentBinding();
      }
    });

    JPanel top = new JPanel(new BorderLayout(0, 6));
    top.setOpaque(false);
    top.add(caption, BorderLayout.NORTH);
    top.add(search, BorderLayout.SOUTH);

    JScrollPane scroll = new JScrollPane(actionList);
    scroll.setBorder(BorderFactory.createLineBorder(Theme.BORDER_STRONG));
    panel.add(top, BorderLayout.NORTH);
    panel.add(scroll, BorderLayout.CENTER);
    return panel;
  }

  private JPanel buildAssignment() {
    JPanel panel = new JPanel();
    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
    panel.setOpaque(false);

    JLabel caption = Ui.caption("Assign to");
    caption.setAlignmentX(LEFT_ALIGNMENT);
    panel.add(caption);
    panel.add(Box.createVerticalStrut(8));

    panel.add(row("Controller", roleBox));
    panel.add(row("Button", inputBox));
    panel.add(row("Hold with", modifierBox));

    JLabel comboNote = Ui.note("\"Hold with\" combines two buttons into one binding. "
        + "Set it to NONE for a plain single-button press.", 320);
    comboNote.setAlignmentX(LEFT_ALIGNMENT);
    panel.add(Box.createVerticalStrut(4));
    panel.add(comboNote);

    JButton apply = Ui.primary("Apply binding", Icons.check(14, Theme.TEXT_ON_ACCENT));
    apply.setAlignmentX(LEFT_ALIGNMENT);
    apply.addActionListener(e -> applyBinding());
    panel.add(Box.createVerticalStrut(10));
    panel.add(apply);

    statusLabel.setForeground(GOOD);
    statusLabel.setFont(Theme.ui(Font.PLAIN, 12f));
    statusLabel.setAlignmentX(LEFT_ALIGNMENT);
    panel.add(Box.createVerticalStrut(8));
    panel.add(statusLabel);

    panel.add(Box.createVerticalGlue());
    return panel;
  }

  private JPanel row(String caption, java.awt.Component field) {
    JPanel row = new JPanel(new BorderLayout(10, 0));
    row.setOpaque(false);
    row.setAlignmentX(LEFT_ALIGNMENT);
    row.setMaximumSize(new Dimension(380, 34));
    row.setBorder(BorderFactory.createEmptyBorder(0, 0, 6, 0));
    JLabel label = Ui.label(caption, Theme.ui(Font.PLAIN, 13f), TEXT);
    label.setPreferredSize(new Dimension(90, 28));
    row.add(label, BorderLayout.WEST);
    row.add(field, BorderLayout.CENTER);
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
