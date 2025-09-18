package frc.robot.Vision;

import java.util.HashSet;
import java.util.LinkedList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class ParentPhotonVisionIO implements BaseVisionIO{
    protected final PhotonCamera camera;
    protected final Transform3d camera_pos;

    //creates camera object
    public ParentPhotonVisionIO(String name, Transform3d camera_pos){
        camera = new PhotonCamera(name);
        this.camera_pos = camera_pos;
    }

    //nearly identical to Limelight reference limelight
    @Override 
    public void update_inputs(BaseVisionIOInput inputs){
        

        //Set<Integer> april_tag_IDs = new HashSet<>();
        //List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        for (var raw_data : camera.getAllUnreadResults()) {
            if (raw_data.hasTargets()){
                
            }
            else{

            }

            
        }
    }
    
}