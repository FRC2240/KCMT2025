package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import frc.robot.utils.ManipulatorState;

public final class Constants {
  public static enum RobotName {
    SABERTOOTH,
    FRIDGE
  };

  public final static class ManipulatorStates {
    public static final ManipulatorState L1 = new ManipulatorState(7.83, -24.9);
    public static final ManipulatorState L2 = new ManipulatorState(0, -10.25);
    public static final ManipulatorState L3 = new ManipulatorState(11.58, -9.9);
    public static final ManipulatorState L4 = new ManipulatorState(28.55, -10.43);
    public static final ManipulatorState IDLE = new ManipulatorState(6.82, -33.47);
    public static final ManipulatorState IDLE_WITH_GP = new ManipulatorState(12.329, 19.7);
    public static final ManipulatorState PROCESSOR = new ManipulatorState(4, -25);
    public static final ManipulatorState ALGAE_L2 = new ManipulatorState(7.35, -16.8);
    public static final ManipulatorState ALGAE_L3 = new ManipulatorState(19.2, -16.8);
    public static final ManipulatorState BARGE = new ManipulatorState(35.1, -5.1);
    public static final ManipulatorState GROUND_ALGAE = new ManipulatorState(0.133, -25.688);

    // States where either the elevator or wrist move before the other
    public static final ManipulatorState[] ELEVATOR_FIRST_STATES = {IDLE, L4};
    public static final ManipulatorState[] WRIST_FIRST_STATES = {L2};
  }

  public static final RobotName ROBOT_NAME = RobotName.SABERTOOTH;
  public static final double MAX_SPEED = 4;

  public static class Funnel {
    public static final int MOTOR_ID = 21;
    public static final int DEFAULT_CURRENT = 3;
  }

  public static class Wrist {
    public static final int MOTOR_ID = 5;
  }

  public static class Grabber {
    public static final int SENSOR_ID = 32;
    public static final int MOTOR_ID = 42;

    // Distance required for a game piece to be "Intaked"
    public static Distance INTAKE_THRESHOLD = Inches.of(2);

    public static final Current INTAKE_CORAL_CURRENT = Amps.of(-50);
    public static final Current INTAKE_ALGAE_CURRENT = Amps.of(-120);
    public static final Current EXTAKE_ALGAE_CURRENT = Amps.of(30);
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.5;
  }
}
