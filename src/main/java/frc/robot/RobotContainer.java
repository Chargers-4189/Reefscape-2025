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
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.OuttakeCoral;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;

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
 /*  private final CommandXboxController secondaryController = new CommandXboxController(
    1
  );*/

  // The robot's subsystems and commands are defined here...
  private final CoralEffector coraleffector = new CoralEffector();
  private final Elevator elevator = new Elevator();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    //System.out.println(AutoBuilder.getAllAutoNames());
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //primaryController.x().onTrue(new MoveElevator(1,elevator,coraleffector));
    primaryController.x().whileTrue(new ManualElevator(elevator));
    elevator.setDefaultCommand(Commands.run(()->{
      elevator.stayStill();
    }, this.elevator));
    primaryController.y().onTrue(new MoveElevator(2,elevator,coraleffector));
    primaryController.b().onTrue(new MoveElevator(3,elevator,coraleffector));
    primaryController.a().onTrue(new MoveElevator(4,elevator,coraleffector));
    //primaryController.x().onTrue(new IntakeCoral(coraleffector));
    //primaryController.a().onTrue(new OuttakeCoral(coraleffector));
    /*primaryController.b().onTrue(new IntakeCoral(coraleffector));
    primaryController.a().onTrue(Commands.run(()->{
      elevator.moveElevator(.5);
      System.out.println("im working");
    }, this.elevator));
    primaryController.a().onFalse(Commands.run(()->{
      elevator.moveElevator(0);
      System.out.println("im working");
    }, this.elevator));**/

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
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