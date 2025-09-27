package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
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
    public static final ManipulatorState INTAKE = new ManipulatorState(0.41, -33.6);
    public static final ManipulatorState PROCESSOR = new ManipulatorState(4, -25);
    public static final ManipulatorState ALGAE_L2 = new ManipulatorState(7.35, -16.8);
    public static final ManipulatorState ALGAE_L3 = new ManipulatorState(19.2, -16.8);
    public static final ManipulatorState BARGE = new ManipulatorState(35.1, -5.1);
    public static final ManipulatorState GROUND_ALGAE = new ManipulatorState(0.133, -25.688);

    public static final Angle POST_SCORE_WRIST_ANGLE = Rotation.of(-16.83);

    // States where either the elevator or wrist move before the other
    public static final ManipulatorState[] ELEVATOR_FIRST_STATES = {IDLE, L4};
    public static final ManipulatorState[] WRIST_FIRST_STATES = {L2, GROUND_ALGAE};
  }

  public static final RobotName ROBOT_NAME = RobotName.SABERTOOTH;
  public static final double MAX_SPEED = 4;

  public static class Funnel {
    public static final int MOTOR_ID = 21;
    public static final int DEFAULT_CURRENT = 3;
  }

  public static class Wrist {
    public static final int MOTOR_ID = 5;

    public static final Angle OFFSET_AMOUNT = Rotations.of(2);
    public static final Angle POSITION_THRESHOLD = Degrees.of(8);
  }

  public static class Elevator {
    public static final Angle OFFSET_AMOUNT = Rotations.of(3);

  }

  public static class Grabber {
    public static final int SENSOR_ID = 32;
    public static final int MOTOR_ID = 42;

    // Distance required for a game piece to be "Intaked"
    public static Distance INTAKE_THRESHOLD = Inches.of(2);

    public static final Current INTAKE_CORAL_CURRENT = Amps.of(-50);
    public static final Current EXTAKE_CORAL_L1_CURRENT = Amps.of(0.5);

    public static final Current INTAKE_ALGAE_CURRENT = Amps.of(-120);
    public static final Current EXTAKE_ALGAE_CURRENT = Amps.of(30);
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.5;
  }

  public static class Vision {
    //stores tag layout for the current year's feild
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //assuming stores distance of cameras from center of bot
    public static Transform3d CAMERA_0_POS = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)); // TBD
    public static Transform3d CAMERA_1_POS = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)); // TBD

    public static double MAX_UNCERTAINTY = 0.0; // TBD
    public static double MAX_Z_ERROR = 0.25; //TBD

    // Standard deviation coefficents, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STDEV_COEFF = 0.02; // Meters
    public static double ANGULAR_STDEV_COEFF = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STDEV_MEGATAG_2_COEFF = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STDEV_MEGATAG_2_COEFF = Double.POSITIVE_INFINITY; // No rotation data available    
  }
}
