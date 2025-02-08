// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagCamera {

  AprilTagFieldLayout tagLayout;

  private boolean estimateAvailable = false;
  public Pose2d estimatedPose;
  public Double estimatedTagYaw;

  protected PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;

  public AprilTagCamera(String cameraName, Transform3d cameraTranslation) {
    camera = new PhotonCamera(cameraName);
    try {
      tagLayout =
        AprilTagFieldLayout.loadFromResource(
          AprilTagFields.k2024Crescendo.m_resourceFile
        );
    } catch (Exception e) {
      System.err.println(e);
    }
    poseEstimator =
      new PhotonPoseEstimator(
        tagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraTranslation
      );
    poseEstimator.setMultiTagFallbackStrategy(
      PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
    );
  }

  public Pose2d getEstimatedRobotPose() {
    estimateAvailable = false;
    return estimatedPose;
  }

  public Double getEstimatedTagYaw() {
    if (!estimateAvailable) {
      return null;
    }
    return estimatedTagYaw;
  }

  public boolean isEstimateReady() {
    return estimateAvailable;
  }

  public void update() {
    var results = camera.getAllUnreadResults();
    var estimatedResult = targetFilter(results);
    if (estimatedResult != null) {
			estimateAvailable = true;
      estimatedPose = estimatedResult.estimatedPose.toPose2d();
    }
  }

  private EstimatedRobotPose targetFilter(List<PhotonPipelineResult> results) {
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      result.targets.removeIf(tag -> {
        double maxDistance = 6.0;
        Transform3d transform = tag.getBestCameraToTarget();
        return (
          transform.getX() > maxDistance || transform.getY() > maxDistance
        );
      });
      estimatedTagYaw = result.getBestTarget().getYaw();
      if (
        result.hasTargets() &&
        result.getTargets().size() < 16 &&
        result.getTargets().size() > 0
      ) {
        var estimatedResult = poseEstimator.update(result);
        if (estimatedResult.isPresent()) {
          return estimatedResult.get();
        }
      }
    }
    return null;
  }
}

