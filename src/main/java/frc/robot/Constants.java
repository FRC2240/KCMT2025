package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.ManipulatorState;

public final class Constants {
  public static enum RobotName {
    SABERTOOTH,
    FRIDGE
  };

  public static enum mode{
    REAL,
    SIM,
    REPLAY
  }

  public static final RobotName ROBOT_NAME = RobotName.FRIDGE;
  public static final mode CURRENT_MODE = (RobotBase.isReal() ? mode.REAL : mode.SIM);
  public static final double MAX_SPEED = 4;

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
    public static final ManipulatorState[] ELEVATOR_FIRST_STATES = { IDLE, L4 };
    public static final ManipulatorState[] WRIST_FIRST_STATES = { L2, GROUND_ALGAE };
  }

  public final static class Alignment {
    // X and Y are meters, rot is degrees
    private static Pose2d createPose(double x, double y, double rot) {
      return new Pose2d(Meters.of(x), Meters.of(y), Rotation2d.fromDegrees(rot));
    }

    // REEF SIDES CODES
    //     3
    //  2 /-\ 4
    //  1 \-/ 5
    //     0
    // ------- DRIVER LINE

    public static final Pose2d REEF_0_LEFT = createPose(2.9237, 4.2409, -3.86);
    public static final Pose2d REEF_0_RIGHT = createPose(2.9237, 3.8894, -2.66);

    public static final Pose2d REEF_1_LEFT = createPose(3.8876, 5.4759, -62.2);
    public static final Pose2d REEF_1_RIGHT = createPose(3.5964, 5.3152, -62.28);

    public static final Pose2d REEF_2_LEFT = createPose(5.4639, 5.2550, -125.15);
    public static final Pose2d REEF_2_RIGHT = createPose(5.1928, 5.4357, -125.31);

    public static final Pose2d REEF_3_LEFT = createPose(6.0563, 3.8392, 179.56);
    public static final Pose2d REEF_3_RIGHT = createPose(6.0563, 4.1605, 179.24);

    public static final Pose2d REEF_4_LEFT = createPose(5.0724, 2.5741, 115.46);
    public static final Pose2d REEF_4_RIGHT = createPose(5.3535, 2.7147, 115.61);

    public static final Pose2d REEF_5_LEFT = createPose(3.5362, 2.7749, 58.19);
    public static final Pose2d REEF_5_RIGHT = createPose(3.8374, 2.5842, 58.65);

    public static final Pose2d[][] REEF_POSITIONS = {
      {REEF_0_LEFT, REEF_0_RIGHT},
      {REEF_1_LEFT, REEF_1_RIGHT},
      {REEF_2_LEFT, REEF_2_RIGHT},
      {REEF_3_LEFT, REEF_3_RIGHT},
      {REEF_4_LEFT, REEF_4_RIGHT},
      {REEF_5_LEFT, REEF_5_RIGHT}
    };

    public static final int LEFT = 0;
    public static final int RIGHT = 1;
  }

  public static class Funnel {
    public static final int MOTOR_ID = 33;
    public static final int DEFAULT_CURRENT = 3;
  }

  public static class Wrist {
    public static final int MOTOR_ID = 5;

    public static final Angle OFFSET_AMOUNT = Rotations.of(2);
    public static final Angle POSITION_THRESHOLD = Degrees.of(8);
  }

  public static class Elevator {
    public static final int LEFT_MOTOR_ID = 20;
    public static final int RIGHT_MOTOR_ID = 21;

    public static final Angle OFFSET_AMOUNT = Rotations.of(3);
    public static final Angle POSITION_THRESHOLD = Degrees.of(4);
    public static final Angle DEFAULT = Rotations.of(0);
  }

  public static class Grabber {
    public static final int SENSOR_ID = 32;
    public static final int MOTOR_ID = 48;

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

  public static class Candle {
    public static final int CANDLE_ID = 10;
    public static final int LED_COUNT = 39;
  }

  public static class Vision {
    //stores tag layout for the current year's feild
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //assuming stores distance of cameras from center of bot
    public static Transform3d CAMERA_0_POS = new Transform3d(-0.272575, 0.2413, 0.520699, new Rotation3d(Degrees.of(0), Degrees.of(-32), Degrees.of(-20))); 
    public static Transform3d CAMERA_1_POS = new Transform3d(0.272575, 0.2413, 0.510699, new Rotation3d(Degrees.of(0), Degrees.of(-32), Degrees.of(20))); 

    public static String CAMERA_0_NAME = "camera_0"; //probably don't change bc network tables
    public static String CAMERA_1_NAME = "camera_1"; //probably don't change bc network tables

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
