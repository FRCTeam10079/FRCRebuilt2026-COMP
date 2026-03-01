package frc.robot.constants;

public class HeadingControllerConstants {
  public static final double SNAP_KP = 0.02;
  public static final double SNAP_KI = 0.0;
  public static final double SNAP_KD = 0.001;

  public static final double MAINTAIN_KP = 0.01;
  public static final double MAINTAIN_KI = 0.0;
  public static final double MAINTAIN_KD = 0.0005;

  public static final double HEADING_TOLERANCE_DEGREES = 2.0;

  // ==================== LAUNCH MODE HEADING CONTROL ====================
  // These are used by the shoot-on-the-move heading controller.
  // The launch controller uses PD + angular velocity feedforward (NOT the
  // SNAP/MAINTAIN PID).
  // Gains are in radians (not degrees) - much more aggressive than SNAP/MAINTAIN.
  //
  // MA (6328) uses kP=8.0, kD=0.5 with radians. We use the same as a starting
  // point.

  /** P gain for launch-mode heading control (rad/s output per radian of error). */
  // TODO: TUNE ON THE ROBOT - start with 8.0 (same as MA), adjust for
  // overshoot/oscillation
  public static final double LAUNCH_KP = 1.0;

  /** D gain for launch-mode heading control (rad/s output per rad/s velocity error). */
  // TODO: TUNE ON THE ROBOT - start with 0.5 (same as MA)
  public static final double LAUNCH_KD = 6.0;

  /**
   * Heading tolerance (degrees) for "on target" during shoot-on-the-move. Wider than static
   * shooting (3 deg) because the heading is continuously tracking a moving target.
   */
  // TODO: TUNE ON THE ROBOT - start at 10 deg, tighten as you gain confidence
  public static final double LAUNCH_HEADING_TOLERANCE_DEGREES = 45.0;

  protected HeadingControllerConstants() {}
}
