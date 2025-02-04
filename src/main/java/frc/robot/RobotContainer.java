// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveController;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.SimpleMoveElevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final Elevator elevator = new Elevator();
  private final CoralEffector coralEffector = new CoralEffector();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(
    0
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*
    new Trigger(m_exampleSubsystem::exampleCondition)
      .onTrue(new ExampleCommand(m_exampleSubsystem));
    */
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driveController
      .a()
      .and(driveController.rightBumper())
      .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driveController
      .b()
      .and(driveController.rightBumper())
      .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driveController
      .x()
      .and(driveController.rightBumper())
      .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driveController
      .y()
      .and(driveController.rightBumper())
      .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    elevator.setDefaultCommand(
      Commands.run(
        () -> elevator.setVoltage(driveController.getLeftY()),
        elevator
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return swerveDrive.followPathCommand("testPath");
  }
}
