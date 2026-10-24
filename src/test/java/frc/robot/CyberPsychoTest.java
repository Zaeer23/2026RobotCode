package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The latch is one-way and the robot loses its mechanisms when it closes, so the thing worth
 * testing is that it closes on exactly the right sequence and nothing else.
 *
 * <p>The state is static and deliberately has no public reset — that is the feature. These tests
 * clear it reflectively, which is a thing a test may do and production code may not.
 */
public class CyberPsychoTest {

  /** up, up, down, down, left, right, left, right */
  private static final int[] CODE = {0, 0, 180, 180, 270, 90, 270, 90};

  private static final int NONE = -1;

  @BeforeEach
  public void reset() throws Exception {
    set("active", false);
    set("progress", 0);
    set("lastPov", -1);
  }

  private static void set(String name, Object value) throws Exception {
    Field field = CyberPsycho.class.getDeclaredField(name);
    field.setAccessible(true);
    field.set(null, value);
  }

  /** Presses a direction and releases it, which is what one D-pad press looks like to the loop. */
  private static void press(int pov) {
    CyberPsycho.update(pov);
    CyberPsycho.update(NONE);
  }

  @Test
  public void theFullSequenceLatchesIt() {
    for (int pov : CODE) {
      press(pov);
    }
    assertTrue(CyberPsycho.isActive(), "the full code should engage it");
  }

  @Test
  public void aPartialSequenceDoesNot() {
    for (int i = 0; i < CODE.length - 1; i++) {
      press(CODE[i]);
    }
    assertFalse(CyberPsycho.isActive(), "seven of eight presses must not engage it");
  }

  @Test
  public void aWrongPressBreaksIt() {
    press(CODE[0]);
    press(CODE[1]);
    press(90);              // right, where down belongs
    for (int i = 2; i < CODE.length; i++) {
      press(CODE[i]);
    }
    assertFalse(CyberPsycho.isActive(), "a wrong press must break the run");
  }

  @Test
  public void holdingADirectionIsOnePressNotMany() {
    // Up held for several loops is one "up". Without edge detection, holding up would satisfy
    // the first two steps on its own and the code would be far easier to enter by accident.
    for (int i = 0; i < 20; i++) {
      CyberPsycho.update(0);
    }
    CyberPsycho.update(NONE);
    for (int i = 2; i < CODE.length; i++) {
      press(CODE[i]);
    }
    assertFalse(CyberPsycho.isActive(), "a held direction must not count as two presses");
  }

  @Test
  public void aFalseStartCanRollIntoARealOne() {
    // "down" then the whole code. The stray press must not poison the attempt that follows.
    press(180);
    for (int pov : CODE) {
      press(pov);
    }
    assertTrue(CyberPsycho.isActive(), "a stray press before the code must not block it");
  }

  @Test
  public void theOperatorPadCanEnterItWhenTheDriverPadIsUnplugged() {
    // An unplugged controller reports -1 forever. This robot has had its driver pad unplugged
    // while every binding on it silently did nothing, so a sequence only that pad could enter
    // would be unenterable exactly when someone wanted it.
    for (int pov : CODE) {
      CyberPsycho.update(NONE, pov);
      CyberPsycho.update(NONE, NONE);
    }
    assertTrue(CyberPsycho.isActive(), "the operator pad alone should be able to engage it");
  }

  @Test
  public void eitherPadCountsForTheSameSequence() {
    // Half on one pad, half on the other. Whoever is holding a working controller can finish it.
    for (int i = 0; i < CODE.length; i++) {
      if (i % 2 == 0) {
        CyberPsycho.update(CODE[i], NONE);
      } else {
        CyberPsycho.update(NONE, CODE[i]);
      }
      CyberPsycho.update(NONE, NONE);
    }
    assertTrue(CyberPsycho.isActive(), "the sequence should not care which pad each press is on");
  }

  @Test
  public void itNeverTurnsItselfOff() {
    for (int pov : CODE) {
      press(pov);
    }
    assertTrue(CyberPsycho.isActive());

    // Everything else that could plausibly happen afterwards, including re-entering the code.
    for (int i = 0; i < 200; i++) {
      CyberPsycho.update(i % 2 == 0 ? NONE : (i * 90) % 360);
    }
    for (int pov : CODE) {
      press(pov);
    }
    assertTrue(CyberPsycho.isActive(), "there is no way out but a reboot");
  }
}
