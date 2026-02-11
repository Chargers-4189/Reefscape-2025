// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CancelAll;
import frc.robot.commands.swervedrive.AlignReef;
import frc.robot.commands.swervedrive.Drive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController primaryController = new CommandXboxController(
    0
  );

  // The robot's subsystems and commands are defined here...
  //private final Climber climber = new Climber();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );
  private final Vision vision = new Vision(
    () -> new Pose2d(),
    drivebase.getSwerveDrive()
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("3 Coral Right");

  public RobotContainer() {
    //System.out.println(AutoBuilder.getAllAutoNames());
    // Configure the trigger bindings
    configureBindings();
    //configureAutoChooser();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    /*
    drivebase.setDefaultCommand(angularVelocityDrive);
    nitroTrigger.and(stationAlign.negate()).and(xFormation.negate()).whileTrue(nitroDrive);
    stationAlign.and(nitroTrigger.negate()).and(xFormation.negate()).whileTrue(stationAngleDrive);
    stationAlign.and(nitroTrigger).and(xFormation.negate()).whileTrue(nitroStationAngleDrive);
    xFormation.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    */
    drivebase.setDefaultCommand(new Drive(
      drivebase,
      primaryController::getLeftX,
      primaryController::getLeftY,
      primaryController::getRightX,
      () -> primaryController.leftStick().getAsBoolean() || primaryController.rightStick().getAsBoolean(),
      primaryController.leftTrigger(.5)::getAsBoolean)
    );
    primaryController.rightTrigger(.5).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    
    primaryController.back().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    primaryController
      .leftBumper()
      .onTrue(Commands.sequence(
        new AlignReef(drivebase, false)
      ));
      primaryController
      .rightBumper()
      .onTrue(Commands.sequence(
        new AlignReef(drivebase, true)
      ));

    
    //primaryController.povUp().onTrue(Commands.run(() -> drivebase.driveToReefClosest(false).withTimeout(.5).schedule(), drivebase));
    /*
    primaryController.rightBumper().onTrue(Commands.sequence(
      Commands.run(() -> drivebase.driveToReefClosest(true).withTimeout(2.5).schedule(), drivebase),
      drivebase.driveForwardRobotRelative(.05).withTimeout(.5)
    ));
    */

    //Cancel All
    primaryController
      .start()
      .onTrue(
        new CancelAll(drivebase)
          .withTimeout(.5)
      );
    }

    //Secondary

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
