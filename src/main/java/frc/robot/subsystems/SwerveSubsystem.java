// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  File swerveJsonDirectory = new File(
      Filesystem.getDeployDirectory(),
      "swerve");
  SwerveDrive swerveDrive;

  /** Creates a new SwerveDrive. */
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // CHANGE TO LOW IN COMP
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(SwerveConstants.kMaxVelocity, new Pose2d(2, 7, new Rotation2d()));
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
        (speeds, feedforwards) -> swerveDrive.drive(speeds),
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
