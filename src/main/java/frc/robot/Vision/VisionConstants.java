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

    //example has a max z error doesn't seem neceary
}