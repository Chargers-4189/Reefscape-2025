package frc.util;


import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;


import frc.robot.subsystems.Vision;

public class Camera {
    private Vision vis = new Vision();
   public PhotonPipelineResult getBestResultRightCam(Vision vis){
    this.vis = vis;
    List <PhotonPipelineResult> results = vis.ResultsRight();
    PhotonPipelineResult bestResult;
        try{
            
            
            bestResult = results.get(0);
           // System.out.println("bestResult: " + bestResult);
            //System.out.println("Running camera");
            return bestResult;
            

            
        }catch(Exception e){

            bestResult = results.get(0);
            return bestResult;
        
        }

        
    
    
   }
   public PhotonPipelineResult getBestResultleftCam(Vision vis){
    this.vis = vis;
    List <PhotonPipelineResult> results = vis.Resultsleft();
    PhotonPipelineResult bestResult;
        try{
            
            
            bestResult = results.get(0);
           // System.out.println("bestResult: " + bestResult);
            //System.out.println("Running camera");
            return bestResult;
            

            
        }catch(Exception e){

            bestResult = results.get(0);
            return bestResult;
        
        }

        
    
    
   }
   
}
