// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants.AlignmentConstants;

public class Swerve extends SubsystemBase {
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  File swerveJsonDirectory = new File(
      Filesystem.getDeployDirectory(),
      "swerve");
  SwerveDrive swerveDrive;

  /** Creates a new SwerveDrive. */
  public Swerve() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // CHANGE TO LOW IN COMP
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(SwerveConstants.kMaxVelocity, SwerveConstants.kINITIAL_POSE);
    } catch (Exception e) {
      System.err.println("SwerveDrive no workie :(");
    }

    // ----- PathPlanner ------
    ModuleConfig swerveModuleConfig = new ModuleConfig(SwerveConstants.kWheelRadius, SwerveConstants.kMaxVelocity,
        SwerveConstants.kWheelCOF, DCMotor.getNeoVortex(1), SwerveConstants.kDriveRatio, SwerveConstants.kDriveAmpLimit,
        1);
    RobotConfig config = new RobotConfig(SwerveConstants.kRobotWeight, SwerveConstants.kMOI, swerveModuleConfig,
        this.swerveDrive.swerveDriveConfiguration.moduleLocationsMeters);

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        () -> {
          return false;
          // Uncomment when using vision or station based control
          // var alliance = DriverStation.getAlliance();
          // if (alliance.isPresent()) {
          // return alliance.get() == DriverStation.Alliance.Red;
          // }
          // return false;
        },
        this);

  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }
  
  public void resetGyro() {
    resetGyro();
    swerveDrive.zeroGyro();
  }

  public void drive(double translationX, double translationY,
  double angularRotationX, boolean fieldOriented) {
    swerveDrive.drive(new Translation2d(-translationX * swerveDrive.getMaximumChassisVelocity(),
          -translationY * swerveDrive.getMaximumChassisVelocity()),
          -angularRotationX * swerveDrive.getMaximumChassisAngularVelocity(),
          fieldOriented,
          false);
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      swerveDrive
          .driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
              headingX.getAsDouble(),
              headingY.getAsDouble(),
              swerveDrive.getOdometryHeading().getRadians(),
              swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX, boolean fieldOriented) {
    return run(() -> {
      swerveDrive.drive(new Translation2d(-translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
          -angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
          fieldOriented,
          false);
    });
  }

  /*
  public Command goToReefByTagNumber(int aprilTagNumber, boolean right) {

    Pose2d targetAprilTagPose = aprilTagFieldLayout.getTagPose(aprilTagNumber).get().toPose2d();
    PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));
    
    double offset = AlignmentConstants.kDIST_OFFSET;
    if (!right) {
      offset *= -1;
    }

    return (
        AutoBuilder.pathfindToPose(
          new Pose2d().transformBy(
            targetAprilTagPose.minus(
              new Pose2d(
                new Translation2d(AlignmentConstants.kDIST_FROM_REEF, offset).rotateBy(targetAprilTagPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-180)))),
                new Rotation2d()
                )
              )
            ),
          constraints
        )
      );
  }*/

  public Command goToReef(boolean right) {
    Pose2d[] tags = {
      aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(11).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
      aprilTagFieldLayout.getTagPose(22).get().toPose2d(),
    };
    Pose2d targetAprilTagPose = getPose().nearest(Arrays.asList(tags));
    PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));
    
    double offset = AlignmentConstants.kDIST_OFFSET;
    if (!right) {
      offset *= -1;
    }

    return (
        AutoBuilder.pathfindToPose(
          new Pose2d().transformBy(
            targetAprilTagPose.minus(
              new Pose2d(
                new Translation2d(AlignmentConstants.kDIST_FROM_REEF, offset).rotateBy(targetAprilTagPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-180)))),
                new Rotation2d()
                )
              )
            ),
          constraints
        )
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
