package frc.robot.constants;

public class HeadingControllerConstants {
  public static final double SNAP_KP = 0.02;
  public static final double SNAP_KI = 0.0;
  public static final double SNAP_KD = 0.001;

  public static final double MAINTAIN_KP = 0.01;
  public static final double MAINTAIN_KI = 0.0;
  public static final double MAINTAIN_KD = 0.0005;

  public static final double HEADING_TOLERANCE_DEGREES = 2.0;

  protected HeadingControllerConstants() {}
}
