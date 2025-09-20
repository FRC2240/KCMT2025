package frc.robot.Vision;

import java.util.Set;

import javax.naming.spi.DirStateFactory.Result;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.HashSet;
import java.util.List;
import java.util.LinkedList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class ParentPhotonVisionIO implements BaseVisionIO{
    protected final PhotonCamera camera;
    protected final Transform3d camera_pos;

    //constructor method for camera objects
    public ParentPhotonVisionIO(String name, Transform3d camera_pos){
        camera = new PhotonCamera(name);
        this.camera_pos = camera_pos;
    }

    //nearly identical to Limelight reference limelight
    @Override 
    public void update_inputs(BaseVisionIOInput inputs){
        //instance variable tells if controller is found
        inputs.controller_found = camera.isConnected();

        Set<Short> april_tag_IDs = new HashSet<>();
        List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        for (var raw_data : camera.getAllUnreadResults()) {
            if (raw_data.hasTargets()){
                // sets instance variable to the angle to tag
                inputs.angle_to_tag = 
                    new rotation(
                        Rotation2d.fromDegrees(raw_data.getBestTarget().getPitch()), 
                        Rotation2d.fromDegrees(raw_data.getBestTarget().getYaw()));
            } else {
                // if no apriltags are found sets angle_to_tag to zero
                inputs.angle_to_tag = new rotation(Rotation2d.kZero, Rotation2d.kZero);
            }

            if (raw_data.multitagResult.isPresent() == true) { 
                //works like raw data in the event of multiple tags
                var multitagResult = raw_data.multitagResult.get();

                // calculate bot pose
                Transform3d feild_to_cam = multitagResult.estimatedPose.best;
                Transform3d feild_to_bot = feild_to_cam.plus(camera_pos.inverse());
                Pose3d bot_pos = new Pose3d(feild_to_bot.getTranslation(), feild_to_bot.getRotation());

                //calculate avg tag distance
                double average_tag_distance = 0.0;
                int i = 0;
                for (var target : raw_data.targets) {
                    average_tag_distance += target.bestCameraToTarget.getTranslation().getNorm();
                    i++;
                }
                average_tag_distance /= i;

                //adds apriltags 
                april_tag_IDs.addAll(multitagResult.fiducialIDsUsed);

                
                pose_estimation_data.add(
                    new pose_estimation_data(

                            raw_data.getTimestampSeconds(), 
                            
                            multitagResult.estimatedPose.ambiguity, 

                            multitagResult.fiducialIDsUsed.size(), 

                            average_tag_distance, 

                            bot_pos, 

                            vision_configuration_type.PHOTOVISION));

            } else if() {

            }
            
        }
    }  
}