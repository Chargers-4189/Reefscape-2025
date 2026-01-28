// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera cam = new PhotonCamera("frCam2025");

  public Vision() {
    //var result = cam.getLatestResult();
    //boolean hasTragets;
    System.out.println(cam.getLatestResult());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = cam.getLatestResult();
    boolean hasTragets = result.hasTargets();
    System.out.println("isHasTarget: " + hasTragets);
    if(hasTragets){
      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();

      // gets the yaw pitch area ID of target to the camera

      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      int targetID = target.getFiducialId(); 
      double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();


      System.out.println("way: " + yaw + " pitch: " + pitch + " area: " + area + " ID: " + targetID + " ambig: " +  poseAmbiguity + " bestCameraToTarget: " + bestCameraToTarget + " alternateCameraToTarget: " + alternateCameraToTarget);

    }else{
      System.out.println("there is no target");
    }
  }
}
