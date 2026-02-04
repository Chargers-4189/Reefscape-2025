package frc.util;


import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;


import frc.robot.subsystems.Vision;

public class Camera {
    private Vision vis = new Vision();
   public PhotonPipelineResult getBestResult(Vision vis){
    this.vis = vis;
    List <PhotonPipelineResult> results = vis.Results();
    PhotonPipelineResult bestResult;

        bestResult = results.get(0);

    return bestResult;
    
    
   }
   
}
