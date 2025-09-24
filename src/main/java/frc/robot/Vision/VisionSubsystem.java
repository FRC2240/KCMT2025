package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.BaseVisionIO.BaseVisionIOInput;

import static frc.robot.vision.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.wpilibj.Alert; disconnected logging

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionSubsystem extends SubsystemBase {

    // empty but can hold an object that implements vision consumer
    private final vision_consumer consumer;
    // empty array that can accept any object implementing the interface
    private final BaseVisionIO[] IO_base;
    //TODO should be AutoLogged auto generated class
    private final BaseVisionIOInput[] input;

    // elipces means multiple objects of vision_IO_Base class can be passed in so
    // multiple cameras
    public VisionSubsystem(vision_consumer consumer, BaseVisionIO... IO_base) {
        // this passes the private final consumer in
        // so running the method uses parameters to define private final variables which
        // then can't be changed

        this.consumer = consumer;
        this.IO_base = IO_base;

        this.input = new BaseVisionIOInput[IO_base.length];
        for (int i = 0; i < IO_base.length; i++) {
            input[i] = new BaseVisionIOInput();
        }

    }

    public Rotation2d getTargetX(int cameraIndex) {
        return input[cameraIndex].angle_to_tag.rot_x();
    }

    // updates input and logs for each camera
    @Override
    public void periodic() {
        for (int i = 0; i < IO_base.length; i++) {
            IO_base[i].update_inputs(input[i]);
            // Logger.processInputs("Vision", input[i]);
        }

        // stores data for all cameras
        List<Pose3d> all_tag_poses = new LinkedList<>(); // position of all tags seen
        List<Pose3d> all_robot_poses = new LinkedList<>(); // all calculated robot poses
        List<Pose3d> all_accepted_poses = new LinkedList<>(); // all accepted (reasonable) positions
        List<Pose3d> all_rejected_poses = new LinkedList<>(); // all rejected (outside theshold unrealistic)

        // does a bunch of suff for each camera
        for (int i = 0; i < IO_base.length; i++) {

            List<Pose3d> tag_poses = new LinkedList<>();
            List<Pose3d> robot_poses = new LinkedList<>();
            List<Pose3d> accepted_poses = new LinkedList<>();
            List<Pose3d> rejected_poses = new LinkedList<>();

            // each tag seen ID appends its position to tag poses
            for (int tag_ID : input[i].april_tag_IDs) {
                var tag_pose = VisionConstants.april_tag_layout.getTagPose(tag_ID);
                if (tag_pose.isPresent()) {
                    tag_poses.add(tag_pose.get());
                }
            }

            //
            for (var estimation : input[i].pose_estimation_data) {
                // confirms estimation data is with in thresholds
                // example has many unecissary seeming conditions
                // come back to
                // potential point of failure
                boolean reject_pose = estimation.april_tag_count() == 0 // rejects estimates made without tags
                        || (estimation.april_tag_count() == 1
                                && estimation.uncertainty() > VisionConstants.max_uncertainty)
                        || (Math.abs(estimation.position().getX())  > VisionConstants.max_z_error)

                        || ((estimation.position().getX() > 0.0)
                            &&  (estimation.position().getX() < april_tag_layout.getFieldLength()))
                        || ((estimation.position().getY() > 0.0) 
                            && (estimation.position().getY() < april_tag_layout.getFieldWidth()));

                robot_poses.add(estimation.position()); // stores all robot positions for a camera
                if (!reject_pose) {
                    accepted_poses.add(estimation.position()); // stores only accepted poses
                } else {
                    rejected_poses.add(estimation.position()); // stores onlt rejected poses
                }

                if (reject_pose) {
                    continue; // skips to next loop
                }

                // calculate stdev
                double stdev_factor = Math.pow(estimation.average_tag_distance(), 2.0)/estimation.april_tag_count();
                


                // my stdev
                
         

                
                //sends vision data
                /* 
                consumer.accepts(
                    estimation.position(),
                    estimation.timestamp(),
                    VecBuilder.fill(, , )
                );
                */

            }

            //logs data by camera
            Logger.recordOutput(
          "Vision/Camera" + Integer.toString(i) + "/Tag_positions",
          tag_poses.toArray(new Pose3d[tag_poses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(i) + "/Robot_positions",
          robot_poses.toArray(new Pose3d[robot_poses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(i) + "/Accepted_position",
          accepted_poses.toArray(new Pose3d[accepted_poses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(i) + "/Rejected_positions",
          rejected_poses.toArray(new Pose3d[rejected_poses.size()]));

            // stores data for each camera
            all_tag_poses.addAll(tag_poses);
            all_robot_poses.addAll(robot_poses);
            all_accepted_poses.addAll(accepted_poses);
            all_rejected_poses.addAll(rejected_poses);

        }

        // logs data

        Logger.recordOutput("Vision/Summary/Tag_positions",
                all_tag_poses.toArray(new Pose2d[all_tag_poses.size()]));
        Logger.recordOutput("Vision/Summary/Robot_positions",
                all_robot_poses.toArray(new Pose2d[all_tag_poses.size()]));
        Logger.recordOutput("Vision/Summary/Accepted_positions",
                all_accepted_poses.toArray(new Pose2d[all_tag_poses.size()]));
        Logger.recordOutput("Vision/Summary/Rejected_positions",
                all_rejected_poses.toArray(new Pose2d[all_tag_poses.size()]));
    }

    // marks a function interface
    // functional interface allows for 1 method here accepts()
    @FunctionalInterface
    // consumer accepts but returns nothing
    public static interface vision_consumer {
        public void accepts(
                double timestamp,
                Pose2d robot_pose,
                // how much uncertainty there is
                // <N3, N1> takes 3 standerd devaitions from vector
                Matrix<N3, N1> stdevs);
    }
}