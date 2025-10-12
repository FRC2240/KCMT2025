package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.ManipulatorState;

public final class Constants {
  public static enum RobotName {
    FRIDGE
  };

  public static enum mode{
    REAL,
  }

  public static final RobotName ROBOT_NAME = RobotName.FRIDGE;
  public static final mode CURRENT_MODE = mode.REAL;
  public static final double MAX_SPEED = 4;

  public final static class ManipulatorStates {
    public static final ManipulatorState L4 = new ManipulatorState(28.55, -10.43); 
    public static final ManipulatorState IDLE = new ManipulatorState(6.82, -33.47);
  }

  public final static class Alignment {
    
    public static final double CONTROLLER_COOLDOWN = 0.3;
    public static final double CONTROLLER_THESHOLD = 0.2;

    public static final PIDController DRIVE_PID_CONTROLLER = new PIDController(5, 0, 0.1);
    public static final PIDController ANGLE_PID_CONTROLLER = new PIDController(5, 0, 0);
    
    public static final Distance MAX_EFFECTIVE_DIST = Meters.of(2.5);
    public static final Distance DISTANCE_THRESHOLD = Inches.of(1);
    public static final Angle ANGLE_THRESHOLD = Degrees.of(10);
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.1;
  }

  public static class Vision {
    //stores tag layout for the current year's feild
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //assuming stores distance of cameras from center of bot
    public static Transform3d CAMERA_0_POS = new Transform3d(-0.272575, 0.2413, 0.520699, new Rotation3d(Degrees.of(0), Degrees.of(-32), Degrees.of(-20))); 
    public static Transform3d CAMERA_1_POS = new Transform3d(0.272575, 0.2413, 0.510699, new Rotation3d(Degrees.of(0), Degrees.of(-32), Degrees.of(20))); 

    public static double MAX_UNCERTAINTY = 0.3; // TBD
    public static double MAX_Z_ERROR = 0.75; //TBD

    // Standard deviation coefficents, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STDEV_COEFF = 0.02; // Meters
    public static double ANGULAR_STDEV_COEFF = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STDEV_MEGATAG_2_COEFF = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STDEV_MEGATAG_2_COEFF = Double.POSITIVE_INFINITY; // No rotation data available    
  }
}
