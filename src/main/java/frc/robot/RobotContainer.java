// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.DriveController;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.MoveElevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Vision vision = new Vision();
  private final Elevator elevator = new Elevator();
  private final CoralEffector coralEffector = new CoralEffector();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(
      Constants.OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*
    swerveDrive.setDefaultCommand(
      new DriveController(swerveDrive, driveController)
    );*/

    /*
    elevator.setDefaultCommand(Commands.run(()->{
      elevator.setVoltage(Constants.ElevatorConstants.kGRAVITY_VOLTS - driveController.getLeftY());
    }, elevator));*/

    //driveController.a().onTrue(new CoralIntake(coralEffector));
    //driveController.b().onTrue(new CoralOuttake(coralEffector));

    driveController.x().whileTrue(new MoveElevator(elevator, 1));
    driveController.y().whileTrue(new MoveElevator(elevator, 2));
    driveController.b().whileTrue(new MoveElevator(elevator,3));
    driveController.a().whileTrue(new MoveElevator(elevator, 4));
    driveController.start().whileTrue(Commands.run(() -> elevator.zeroEncoder()));




    driveController.leftBumper().onTrue(new AutoAlign(swerve, vision, false));
    driveController.rightBumper().onTrue(new AutoAlign(swerve, vision, true));

    //driveController.rightTrigger().onTrue(new AutoAlignIntake(swerve, vision));
    driveController.start().debounce(1).onTrue(Commands.runOnce(()->{swerve.resetGyro();}, swerve));
    //driveController.povUp().onTrue(new INPUTCLIMBCOMMANDUP));
    //driveController.povUpRight().onTrue(new INPUTCLIMBCOMMANDUP));
    //driveController.povUpLeft().onTrue(new INPUTCLIMBCOMMANDUP));
    //driveController.povDown().onTrue(new INPUTCLIMBCOMMANDDon));
    //driveController.povDownRight().onTrue(new INPUTCLIMBCOMMANDDon));
    //driveController.povDownLeft().onTrue(new INPUTCLIMBCOMMANDDon));


    swerve.setDefaultCommand(swerve.driveCommand(() -> driveController.getLeftY(),
        () -> driveController.getLeftX(), () -> driveController.getRightX(), false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("test-001");
  }
}
