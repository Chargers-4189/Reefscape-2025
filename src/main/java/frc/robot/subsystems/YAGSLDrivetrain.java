// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class YAGSLDrivetrain extends SubsystemBase {

  /** Creates a new YAGSLDrivetrain. */

  double maximumSpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond;
  File swerveJsonDirectory = new File(
    Filesystem.getDeployDirectory(),
    "swerve"
  );
  SwerveDrive swerveDrive;

  public YAGSLDrivetrain() {
    try {
      swerveDrive =
        new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(maximumSpeed, new Pose2d(Meter.of(2), Meter.of(7), new Rotation2d()));
    } catch (Exception e) {
      System.err.println("SwerveDrive no workie :(");
    }
    /*
     * Wheel COF = https://docs.google.com/spreadsheets/d/1e-PpfiaOBn0BW1PxHVpOaZREy2jI7hNt-4gQEwFrpzM/edit?gid=1799070435#gid=1799070435
     */
    ModuleConfig moduleConfig = new ModuleConfig(
      Constants.ModuleConstants.kWheelDiameterMeters/2,
      AutoConstants.kMaxSpeedMetersPerSecond,
      .87,
      DCMotor.getNeoVortex(1),
      Constants.ModuleConstants.kDrivingMotorReduction,
      Constants.NeoMotorConstants.kMaxCurrentDriveMotor,
      1
    );
    /*
     * Robot Configuration and Details about variables
     * 100 Pounds = 45.3592 kilograms
     * MOI = 15 inches to 0.381m^2 * 45.3592 kg = 6.5843868312 kg*m^2
     */
    RobotConfig config = new RobotConfig(
      45.592,
      6.5843868312,
      moduleConfig,
      new Translation2d(
        Constants.DriveConstants.kWheelBase / 2,
        Constants.DriveConstants.kTrackWidth / 2
      ),
      new Translation2d(
        Constants.DriveConstants.kWheelBase / 2,
        -Constants.DriveConstants.kTrackWidth / 2
      ),
      new Translation2d(
        -Constants.DriveConstants.kWheelBase / 2,
        Constants.DriveConstants.kTrackWidth / 2
      ),
      new Translation2d(
        -Constants.DriveConstants.kWheelBase / 2,
        -Constants.DriveConstants.kTrackWidth / 2
      )
    );
    AutoBuilder.configure(
      swerveDrive::getPose,
      this::resetOdometry,
      swerveDrive::getRobotVelocity,
      (speeds, feedforwards) -> swerveDrive.drive(speeds),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      () -> {
        return true;
      },
      this
    );
  }

  public void drive(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Command getAutonomousCommmand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  public Command followPathCommand(String pathFile) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathFile);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return new PathPlannerAuto("auto-test");
    } catch (Exception e) {
      DriverStation.reportError(
        "Big oops: " + e.getMessage(),
        e.getStackTrace()
      );
      return Commands.none();
    }
  }

  public Command driveWithControllerCommand(CommandXboxController controller) {
    SwerveInputStream driveSpeeds = SwerveInputStream
      .of(
        swerveDrive,
        () -> controller.getLeftY() * -1,
        () -> controller.getLeftX() * -1
      ) // Axis which give the desired translational angle and speed.
      .withControllerRotationAxis(controller::getRightX) // Axis which give the desired angular velocity.
      .deadband(0.01) // Controller deadband
      .scaleTranslation(0.8) // Scaled controller translation axis
      .allianceRelativeControl(false); // Alliance relative controls.
    //.withControllerHeadingAxis(controller::getRightX, controller::getRightY) // Axis which give the desired heading angle using trigonometry.
    //.headingWhile(true); // Enable heading based control.

    return Commands.run(
      () -> {
        swerveDrive.drive(driveSpeeds.get());
      },
      this
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
