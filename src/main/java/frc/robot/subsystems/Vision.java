// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.AprilTagCamera;

public class Vision extends SubsystemBase {

  private Pose2d avgEstimatedRobotPosition = new Pose2d();
  private boolean avgEstimateAvailable = false;
  private NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private StructPublisher<Pose2d> photonRobotPosition = networkTable
      .getStructTopic("PhotonRobotPosition", Pose2d.struct).publish();

  /** Creates a new vision. */
  private AprilTagCamera[] cameras;

  public Vision() {

    final Rotation3d flCamRot = new Rotation3d(0, Math.toRadians(0), 0);
    final Transform3d flCamPose = new Transform3d(
      new Translation3d(0, -0.01, 0),
      flCamRot
    );
    final Rotation3d frCamRot = new Rotation3d(0, Math.toRadians(0), 0);
    final Transform3d frCamPose = new Transform3d(
      new Translation3d(0, 0, 0),
      frCamRot
    );
    final Rotation3d bkCamRot = new Rotation3d(0, Math.toRadians(0), 0);
    final Transform3d bkCamPose = new Transform3d(
      new Translation3d(0, 0.01, 0),
      bkCamRot
    );

    cameras =
      new AprilTagCamera[] {
        new AprilTagCamera("flCam2025", flCamPose),
        new AprilTagCamera("frCam2025", frCamPose),
        new AprilTagCamera("bkCam2025", bkCamPose),
      };
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

  @Override
  public void periodic() {
    for (AprilTagCamera camera : cameras) {
      camera.update();
    }
    AvgEstimatedRobotPosition();
    photonRobotPosition.set(avgEstimatedRobotPosition);
  }
}
