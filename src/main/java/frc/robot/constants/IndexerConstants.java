package frc.robot.constants;

public class IndexerConstants {
  public static final int kFeederMotorID = 15;
  public static final int kSpindexerMotorID = 21;

  public static final int kCurrentLimit = 40;

  public static final double kFeederKP = 0.5;
  public static final double kFeederKI = 0.0;
  public static final double kFeederKD = 0.0;
  public static final double kFeederKS = 0.5;
  public static final double kFeederKV = 0.3;
  public static final double kFeederKA = 0.01;
  public static final double kFeederKG = 0.0;

  public static final double kSpindexerKP = 0.3;
  public static final double kSpindexerKI = 0.0;
  public static final double kSpindexerKD = 0.0;
  public static final double kSpindexerKS = 0.2;
  public static final double kSpindexerKV = 0.12;
  public static final double kSpindexerKA = 0.01;
  public static final double kSpindexerKG = 0.0;

  public static final double kFeederTargetRPM = 3000;
  public static final double kFeederReverseRPM = -3000;
  public static final double kSpindexerTargetRPM = 5000;
  public static final double kSpindexerReverseRPM = -5000;

  protected IndexerConstants() {}
}
