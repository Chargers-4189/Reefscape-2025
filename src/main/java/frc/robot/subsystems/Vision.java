// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import javax.sound.sampled.SourceDataLine;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera cam = new PhotonCamera("frCam2025");

  public Vision() {
   
    //boolean hasTragets;
    
  }

  public List <PhotonPipelineResult> Results(){
     System.out.println("Running vision");
    var results = cam.getAllUnreadResults();
    //System.out.println("Running vision");
    return results;
  } 

  public void poseEstmater(){

  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = cam.getLatestResult();
    boolean hasTragets = result.hasTargets();
    System.out.println("isHasTarget: " + hasTragets);
    if(hasTragets){
      /* List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();

      // gets the yaw pitch area ID of target to the camera

      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      int targetID = target.getFiducialId(); 
      double poseAmbiguity = target.getPoseAmbiguity();
      

      System.out.println("yaw: " + yaw);
      System.out.println("pitch: " + pitch);
      System.out.println("area: " + area);
      System.out.println("targetID: " + targetID);
      System.out.println("poseAmbiguity: " + poseAmbiguity);

      System.out.println(cam.getAllUnreadResults());
      System.out.println(Results());

    }else{
      System.out.println("there is no target");
      */
    } 
      
    

  }
}
