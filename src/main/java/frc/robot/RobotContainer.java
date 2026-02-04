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
import frc.robot.commands.IntakeDownAuto;
import frc.robot.commands.IntakeDownManual;
import frc.robot.commands.IntakeUpAuto;
import frc.robot.commands.IntakeUpManual;
import frc.robot.subsystems.Intake;

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
  private final CommandXboxController secondaryController = new CommandXboxController(
    1
  );

  // The robot's subsystems and commands are defined here...

  private final Intake Intake = new Intake();
  //private final Climber climber = new Climber();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  //private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("3 Coral Right");

  public RobotContainer() {
    //System.out.println(AutoBuilder.getAllAutoNames());
    // Configure the trigger bindings
    configureBindings();
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
    
    //Change to seconday later...
    
    //Test to make sure they stop at sensor
    primaryController.leftBumper().onTrue(new IntakeDownAuto(Intake));
    primaryController.rightBumper().onTrue(new IntakeUpAuto(Intake));
    //Test to make sure it doesnt kill itself
    primaryController.leftTrigger().whileTrue(new IntakeDownManual(Intake));
    primaryController.rightTrigger().whileTrue(new IntakeUpManual(Intake));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Command() {
      
    };
  }
}