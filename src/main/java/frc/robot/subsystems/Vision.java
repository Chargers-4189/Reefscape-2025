// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.Camera;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision extends SubsystemBase {

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */

    private final NetworkTableInstance networkTable = NetworkTableInstance
    .getDefault()
    .getTable("SwerveSubsystem")
    .getInstance();
  private final StructPublisher<Transform3d> publisher = networkTable
    .getStructTopic("AprilTagPose3D", Transform3d.struct)
    .publish();
    private final StructPublisher<Pose3d> publisher2 = networkTable
    .getStructTopic("GoalPose", Pose3d.struct)
    .publish();

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
    AprilTagFields.k2025ReefscapeWelded
  );
  public VisionSystemSim visionSim;
  private Supplier<Pose2d> currentPose;
  private Field2d field2d;
  private SwerveDrive swerve;

  private Camera[] cameras = {
    new Camera(
      "flCam2025",
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)),
      new Translation3d(
        Units.inchesToMeters(12.25),
        Units.inchesToMeters(6.25),
        Units.inchesToMeters(11.375)
      ),
      VecBuilder.fill(.5, .5, 1),
      VecBuilder.fill(.25, .25, .5)
    ),
    new Camera(
      "frCam2025",
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)),
      new Translation3d(
        Units.inchesToMeters(12.375),
        Units.inchesToMeters(-6.375),
        Units.inchesToMeters(11)
      ),
      VecBuilder.fill(.5, .5, 1),
      VecBuilder.fill(.25, .25, .5)
    )
  };

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, SwerveDrive swerve) {
    this.currentPose = currentPose;
    this.swerve = swerve;
    this.field2d = swerve.field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Camera c : cameras) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException(
        "Cannot get AprilTag " +
        aprilTag +
        " from field " +
        fieldLayout.toString()
      );
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void 
  
  updatePoseEstimation(SwerveDrive swerveDrive) {
    if (
      SwerveDriveTelemetry.isSimulation &&
      swerveDrive.getSimulationDriveTrainPose().isPresent()
    ) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Camera camera : cameras) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        //System.out.println(pose.estimatedPose.toPose2d());
        swerveDrive.addVisionMeasurement(
          pose.estimatedPose.toPose2d(),
          pose.timestampSeconds, //Timer.getFPGATimestamp(),
          camera.curStdDevs
        );
      }
      //swerveDrive.setVisionMeasurementStdDevs(null);
    }
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
        est ->
          debugField
            .getObject("VisionEstimation")
            .setPose(est.estimatedPose.toPose2d()),
        () -> {
          debugField.getObject("VisionEstimation").setPoses();
        }
      );
    }
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag
      .map(pose3d ->
        PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())
      )
      .orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Camera camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews() {
    if (
      Desktop.isDesktopSupported() &&
      Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)
    ) {
      //      try
      //      {
      //        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      //        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      //        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      //      } catch (IOException | URISyntaxException e)
      //      {
      //        e.printStackTrace();
      //      }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {
    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Camera c : cameras) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout
          .getTagPose(target.getFiducialId())
          .get()
          .toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  public Transform3d getLeftTagPose() {
    return cameras[0].getEstimateTagPose();
  }

  public Transform3d getRightTagPose() {
    return cameras[1].getEstimateTagPose();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerve.updateOdometry();
    updatePoseEstimation(swerve);
  }
}
