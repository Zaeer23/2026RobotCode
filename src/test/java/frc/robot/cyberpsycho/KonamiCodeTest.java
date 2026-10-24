package frc.robot.cyberpsycho;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

/**
 * CyberPsycho cannot be undone once it fires, so the code has to be easy to enter on purpose and
 * impossible to enter by accident. These drive the detector the way a D-pad does: one POV sample
 * every 20 ms loop, with the pad held for a few loops on each press.
 */
public class KonamiCodeTest {

  private static final int UP = 0;
  private static final int RIGHT = 90;
  private static final int DOWN = 180;
  private static final int LEFT = 270;
  private static final int CENTRE = -1;

  private static final int[] CODE = {UP, UP, DOWN, DOWN, LEFT, RIGHT, LEFT, RIGHT};

  private static final double LOOP = 0.02;

  /** Feeds samples one loop apart and counts how many times the code fired. */
  private static final class Pad {
    final KonamiCode code = new KonamiCode();
    double now = 0.0;
    int fired = 0;

    void sample(int pov) {
      if (code.update(pov, now)) {
        fired++;
      }
      now += LOOP;
    }

    /** A press held for a few loops, then released for a few: a normal tap. */
    void tap(int pov) {
      for (int i = 0; i < 4; i++) {
        sample(pov);
      }
      for (int i = 0; i < 4; i++) {
        sample(CENTRE);
      }
    }

    void tapAll(int... povs) {
      for (int pov : povs) {
        tap(pov);
      }
    }

    void idle(double seconds) {
      now += seconds;
    }
  }

  @Test
  void theCodeFiresOnce() {
    Pad pad = new Pad();
    pad.tapAll(CODE);
    assertEquals(1, pad.fired);
  }

  @Test
  void itFiresOnTheFinalPressNotTheRelease() {
    Pad pad = new Pad();
    pad.tapAll(UP, UP, DOWN, DOWN, LEFT, RIGHT, LEFT);
    pad.sample(RIGHT);
    assertEquals(1, pad.fired, "should fire the loop the last RIGHT lands");
  }

  @Test
  void holdingTheLastPressDoesNotFireAgain() {
    Pad pad = new Pad();
    pad.tapAll(UP, UP, DOWN, DOWN, LEFT, RIGHT, LEFT);
    for (int i = 0; i < 100; i++) {
      pad.sample(RIGHT);
    }
    assertEquals(1, pad.fired);
  }

  @Test
  void holdingADirectionIsOnePress() {
    // UP held for a full second is one UP, so UP-held + DOWN DOWN ... is one UP short.
    Pad pad = new Pad();
    for (int i = 0; i < 50; i++) {
      pad.sample(UP);
    }
    pad.sample(CENTRE);
    pad.tapAll(DOWN, DOWN, LEFT, RIGHT, LEFT, RIGHT);
    assertEquals(0, pad.fired);
  }

  @Test
  void sloppyDiagonalsDoNotCountAsExtraPresses() {
    // A thumb rolling slightly off UP and back reads UP, UP-RIGHT, UP. That is still one UP.
    Pad pad = new Pad();
    pad.sample(UP);
    pad.sample(45);
    pad.sample(UP);
    pad.sample(CENTRE);
    pad.tap(UP);
    pad.sample(DOWN);
    pad.sample(135);
    pad.sample(DOWN);
    pad.sample(CENTRE);
    pad.tapAll(DOWN, LEFT, RIGHT, LEFT, RIGHT);
    assertEquals(1, pad.fired);
  }

  @Test
  void strayPressesBeforeTheCodeDoNotMatter() {
    Pad pad = new Pad();
    pad.tapAll(LEFT, DOWN, UP, RIGHT);
    pad.tapAll(CODE);
    assertEquals(1, pad.fired);
  }

  @Test
  void aWrongPressSpoilsTheAttempt() {
    Pad pad = new Pad();
    pad.tapAll(UP, UP, DOWN, DOWN, LEFT, LEFT, LEFT, RIGHT);
    assertEquals(0, pad.fired);
  }

  @Test
  void aSpoiledAttemptCanBeRetriedStraightAway() {
    Pad pad = new Pad();
    pad.tapAll(UP, UP, DOWN, RIGHT);
    pad.tapAll(CODE);
    assertEquals(1, pad.fired);
  }

  @Test
  void anExtraUpAtTheStartStillWorks() {
    // Three UPs then the rest: the last eight presses are still the code.
    Pad pad = new Pad();
    pad.tap(UP);
    pad.tapAll(CODE);
    assertEquals(1, pad.fired);
  }

  @Test
  void aLongPauseStartsTheAttemptOver() {
    Pad pad = new Pad();
    pad.tapAll(UP, UP, DOWN, DOWN);
    pad.idle(KonamiCode.MAX_GAP_SECONDS + 0.1);
    pad.tapAll(LEFT, RIGHT, LEFT, RIGHT);
    assertEquals(0, pad.fired);
  }

  @Test
  void aShortPauseIsFine() {
    Pad pad = new Pad();
    pad.tapAll(UP, UP, DOWN, DOWN);
    pad.idle(KonamiCode.MAX_GAP_SECONDS - 0.3);
    pad.tapAll(LEFT, RIGHT, LEFT, RIGHT);
    assertEquals(1, pad.fired);
  }

  @Test
  void pressesSpreadAcrossAMatchNeverAddUp() {
    // The right presses in the right order, but each one seconds apart, as diagnostic presses
    // would be during a match.
    Pad pad = new Pad();
    for (int pov : CODE) {
      pad.tap(pov);
      pad.idle(5.0);
    }
    assertEquals(0, pad.fired);
  }

  @Test
  void theReversedCodeDoesNothing() {
    Pad pad = new Pad();
    pad.tapAll(RIGHT, LEFT, RIGHT, LEFT, DOWN, DOWN, UP, UP);
    assertEquals(0, pad.fired);
  }

  @Test
  void enteringItTwiceFiresTwice() {
    // The detector reports each completed code. Making the mode one-way is the latch's job, and
    // a detector that went deaf after one success would hide a latch that was not latching.
    Pad pad = new Pad();
    pad.tapAll(CODE);
    pad.tapAll(CODE);
    assertEquals(2, pad.fired);
  }

  @Test
  void anUnpluggedPadReadsCentredAndDoesNothing() {
    KonamiCode code = new KonamiCode();
    for (int i = 0; i < 500; i++) {
      assertFalse(code.update(CENTRE, i * LOOP));
    }
  }

  @Test
  void thePressRecordStartsEmpty() {
    // Seven correct presses from cold are not enough; it needs all eight.
    Pad pad = new Pad();
    pad.tapAll(UP, DOWN, DOWN, LEFT, RIGHT, LEFT, RIGHT);
    assertEquals(0, pad.fired);
    pad.tapAll(CODE);
    assertEquals(1, pad.fired);
  }
}
