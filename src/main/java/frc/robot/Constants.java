package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static enum RobotName {
    SABERTOOTH,
    FRIDGE
  };

  public static final RobotName ROBOT_NAME = RobotName.SABERTOOTH;
  public static final double MAX_SPEED = 4;

  public static class Funnel {
    public static final int MOTOR_ID = 21;
    public static final int DEFAULT_CURRENT = 3;
  }

  public static class Grabber {
    public static final int SENSOR_ID = 32;
    public static final int MOTOR_ID = 42;

    // Distance required for a game piece to be "Intaked"
    public static Distance INTAKE_THRESHOLD = Inches.of(2);

    public static final Current INTAKE_CORAL_CURRENT = Current.ofBaseUnits(-50, Amps);
    public static final Current INTAKE_ALGAE_CURRENT = Current.ofBaseUnits(-120, Amps);
    public static final Current EXTAKE_ALGAE_CURRENT = Current.ofBaseUnits(30, Amps);
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.5;
  }
}
