package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
    //stores tag layout for the current year's feild
    public static AprilTagFieldLayout april_tag_layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //assuming stores distance of cameras from center of bot
    public static Transform3d camera_0_pos = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)); // TBD
    public static Transform3d camera_1_pos = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)); // TBD

    public static double max_uncertainty = 0.0; // TBD
    public static double max_z_error = 0.25; //TBD

    // Standard deviation coefficents, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linear_stdev_coeff = 0.02; // Meters
    public static double angular_stdev_coeff = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linear_stdev_MEGATAG_2_coeff = 0.5; // More stable than full 3D solve
    public static double angular_stdev_MEGATAG_2_coeff = Double.POSITIVE_INFINITY; // No rotation data available

}