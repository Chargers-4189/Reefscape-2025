// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class YAGSLDrivetrain extends SubsystemBase {
  /** Creates a new YAGSLDrivetrain. */

  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  public YAGSLDrivetrain() {
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, new Pose2d(5, 5, new Rotation2d()));
    } catch(Exception e) {
      System.err.println("SwerveDrive no workie :(");
    }
  }

  public void drive(ChassisSpeeds speeds){
    swerveDrive.drive(speeds);
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }


  public Command driveWithControllerCommand(CommandXboxController controller){
     SwerveInputStream driveSpeeds = SwerveInputStream.of(swerveDrive,
                                                               () -> controller.getLeftY() * -1,
                                                               () -> controller.getLeftX() * -1) // Axis which give the desired translational angle and speed.
                                                           .withControllerRotationAxis(controller::getRightX) // Axis which give the desired angular velocity.
                                                           .deadband(0.01)                  // Controller deadband
                                                           .scaleTranslation(0.8)           // Scaled controller translation axis
                                                           .allianceRelativeControl(true)  // Alliance relative controls.
                                                           .withControllerHeadingAxis(controller::getRightX,
                                                                                     controller::getRightY) // Axis which give the desired heading angle using trigonometry.
                                                          .headingWhile(true); // Enable heading based control.

    return Commands.run(()->{
      swerveDrive.drive(driveSpeeds.get());
    }, this);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
