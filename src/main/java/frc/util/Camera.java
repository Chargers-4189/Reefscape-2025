package frc.util;


import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;


import frc.robot.subsystems.Vision;

public class Camera {
   public PhotonPipelineResult getBestResult(Vision vis){
    List <PhotonPipelineResult> results = vis.Results();
    PhotonPipelineResult bestResult;

        bestResult = results.get(0);

    return bestResult;
    
    
   }
   
}
