package frc.robot;

public final class Constants {
  public static enum RobotName {
    SABERTOOTH,
    FRIDGE
  };

  public static final RobotName ROBOT_NAME = RobotName.SABERTOOTH;
  public static final double MAX_SPEED = 4;

  public static class Funnel {
    public static final int MOTOR_ID = 21;
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.5;
  }
}
