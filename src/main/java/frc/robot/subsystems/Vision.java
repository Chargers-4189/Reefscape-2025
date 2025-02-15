// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.AprilTagCamera;
import frc.util.AprilTagCamera.AprilTagCameraSim;

public class Vision extends SubsystemBase {

  private Pose2d robotPose = new Pose2d();
  private Pose2d avgEstimatedRobotPosition = new Pose2d();
  private boolean avgEstimateAvailable = false;
  private NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private StructPublisher<Pose2d> photonRobotPosition = networkTable
      .getStructTopic("PhotonRobotPosition", Pose2d.struct).publish();

  private AprilTagCamera[] cameras;
  final Transform3d flCamPose = new Transform3d(
      new Translation3d(0.5, -0.3, 0),
      new Rotation3d(0, Math.toRadians(0), 0));
  final Transform3d frCamPose = new Transform3d(
      new Translation3d(0, 0.3, 0),
      new Rotation3d(0, Math.toRadians(0), 0));
  final Transform3d bkCamPose = new Transform3d(
      new Translation3d(-0.5, 0, 0),
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(180.0)));

  // ----- Simulated Vision -----
  VisionSystemSim visionSim;

  /** Creates a new vision. */
  public Vision() {
    if (Robot.isSimulation()) {
      setupSimCameras();
      System.out.println("SIM");
    } else {
      cameras = new AprilTagCamera[] {
          new AprilTagCamera("flCam2025", flCamPose),
          new AprilTagCamera("frCam2025", frCamPose),
          new AprilTagCamera("bkCam2025", bkCamPose),
      };
    }
  }

  public void setupSimCameras() {
    visionSim = new VisionSystemSim("main");
    try {
      AprilTagFieldLayout tagLayout = AprilTagFieldLayout
          .loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      visionSim.addAprilTags(tagLayout);
    } catch (Exception e) {
      System.out.println("Could not load simulated field: " + e);
    }
    cameras = new AprilTagCamera[] {
        new AprilTagCameraSim("flCam2025", flCamPose, true, visionSim),
        new AprilTagCameraSim("frCam2025", frCamPose, false, visionSim),
        new AprilTagCameraSim("bkCam2025", bkCamPose, false, visionSim),
    };
  }

  public Double getFrontLeftTagYaw() {
    if (cameras[0].getEstimatedTagYaw() != null) {
      return cameras[0].getEstimatedTagYaw();
    } else {
      return null;
    }
  }

  public Double getFrontRightTagYaw() {
    if (cameras[1].getEstimatedTagYaw() != null) {
      return cameras[1].getEstimatedTagYaw();
    } else {
      return null;
    }
  }

  public Double getBackTagYaw() {
    if (cameras[2].getEstimatedTagYaw() != null) {
      return cameras[2].getEstimatedTagYaw();
    } else {
      return null;
    }
  }

  public Pose2d getEstimatedRobotPosition() {
    if (avgEstimateAvailable) {
      return avgEstimatedRobotPosition;
    }
    return null;
  }

  public void AvgEstimatedRobotPosition() {
    int count = 0;
    double x = 0;
    double y = 0;
    double degrees = 0;
    for (AprilTagCamera camera : cameras) {
      if (camera.isEstimateReady()) {
        x += camera.getEstimatedRobotPose().getX();
        y += camera.getEstimatedRobotPose().getY();
        degrees += camera.getEstimatedRobotPose().getRotation().getDegrees();
        count++;
      }
    }
    x /= count;
    y /= count;
    degrees = ((degrees / count) * Math.PI) / 180;
    avgEstimatedRobotPosition = new Pose2d(x, y, new Rotation2d(degrees));
    avgEstimateAvailable = true;
    if (count == 0) {
      avgEstimatedRobotPosition = null;
      avgEstimateAvailable = false;
    }
  }

  public void updatePose(Pose2d pose) {
    robotPose = pose;
  }

  @Override
  public void periodic() {
    for (AprilTagCamera camera : cameras) {
      camera.update();
    }
    if (Robot.isSimulation()) {
      visionSim.update(robotPose);
    }
    AvgEstimatedRobotPosition();
    photonRobotPosition.set(avgEstimatedRobotPosition);
    System.out.println(cameras[0].getEstimatedTagYaw() + " " + cameras[1].getEstimatedTagYaw());
  }
}
