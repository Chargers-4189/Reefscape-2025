// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActuateIntakeDown;
import frc.robot.commands.ActuateIntakeUp;
import frc.robot.commands.AutoPlaceCoral;
import frc.robot.commands.CancelAll;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.util.Elastic;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodi+c methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Elevator elevator = new Elevator();
  private final CoralEffector coralEffector = new CoralEffector();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(
    Constants.OperatorConstants.kDriverControllerPort
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    Elastic.initialize();

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
    coralEffector.setDefaultCommand(new IntakeCoral(coralEffector));

    swerve.setDefaultCommand(
      swerve.driveCommand(
        () -> driveController.getLeftY() * .7,
        () -> driveController.getLeftX() * .7,
        () -> driveController.getRightX() * .7,
        true
      )
    );

    driveController.leftTrigger(.8).and(!effectorForwardTrigger.getAsBoolean()).onTrue(new ActuateIntakeDown(intake));
    driveController.rightTrigger(.8).and(!effectorForwardTrigger.getAsBoolean()).onTrue(new ActuateIntakeUp(intake));

<<<<<<< HEAD
    //driveController.leftTrigger(0.5).onTrue(new OuttakeCoral(coralEffector));
    //driveController.rightTrigger(.5).whileTrue(new ActuateIntakeDown(intake));

    driveController.x().and(!elevatorControlTriggerUp.getAsBoolean() && !elevatorControlTriggerDown.getAsBoolean()).onTrue(new AutoPlaceCoral(vision, elevator, coralEffector, 1));
    driveController.y().and(!elevatorControlTriggerUp.getAsBoolean() && !elevatorControlTriggerDown.getAsBoolean()).onTrue(new AutoPlaceCoral(vision, elevator, coralEffector, 2));
    driveController.b().and(!elevatorControlTriggerUp.getAsBoolean() && !elevatorControlTriggerDown.getAsBoolean()).onTrue(new AutoPlaceCoral(vision, elevator,coralEffector, 3));
    driveController.a().and(!elevatorControlTriggerUp.getAsBoolean() && !elevatorControlTriggerDown.getAsBoolean()).onTrue(new AutoPlaceCoral(vision, elevator, coralEffector, 4));

    driveController.leftBumper().onTrue(new AlignReef(swerve, vision, false).withTimeout(3));
    driveController.rightBumper().onTrue(new AlignReef(swerve, vision, true).withTimeout(3));

    final Trigger elevatorControlTriggerUp = new Trigger(()->(secondaryController.getLeftY() > Constants.OperatorConstants.kSecondaryDeadband)); 
    final Trigger elevatorControlTriggerDown = new Trigger(()->(secondaryController.getLeftY() < -Constants.OperatorConstants.kSecondaryDeadband)); 

    final Trigger effectorForwardTrigger = new Trigger(() ->(secondaryController.getLeftTriggerAxis() < Constants.OperatorConstants.kSecondaryDeadband));
    final Trigger effectorBackTrigger = new Trigger(() ->(secondaryController.getRightTriggerAxis() < Constants.OperatorConstants.kSecondaryDeadband));
  
    final Trigger chuteUpTrigger = new Trigger(() ->(secondaryController.getRightY() > Constants.OperatorConstants.kSecondaryDeadband));
    final Trigger chuteDownTrigger = new Trigger(() ->(secondaryController.getRightY() < -Constants.OperatorConstants.kSecondaryDeadband));

    elevatorControlTriggerUp.onTrue(Commands.run(()->{
      elevator.setVoltage(secondaryController.getLeftY() * 4);
    },elevator));
    elevatorControlTriggerDown.onTrue(Commands.run(()->{
      elevator.setVoltage(secondaryController.getLeftY() * 4);
    },elevator));
    
    effectorBackTrigger.onTrue(Commands.run(()-> {
      coralEffector.setPower(secondaryController.getRightTriggerAxis());
    },coralEffector));
    effectorForwardTrigger.onTrue(Commands.run(()->{
      coralEffector.setPower(secondaryController.getLeftTriggerAxis());
    },coralEffector));

    chuteUpTrigger.onTrue(Commands.run(()->{
      intake.setPower(secondaryController.getRightY());
    },intake));
    chuteDownTrigger.onTrue(Commands.run(() -> {
      intake.setPower(secondaryController.getRightY());
    },intake));
    driveController
      .back()
      .onTrue(new CancelAll(coralEffector, elevator, intake, swerve));
    driveController
      .start()
      .debounce(1)
      .onTrue(
        Commands.runOnce(
          () -> {
            swerve.zeroGyro();
          },
          swerve
        )
      );

    /*
    elevator.setDefaultCommand(Commands.run(()->{
      elevator.setVoltage(-Math.pow(driveController.getRightY(), 3) * 1.5);
    }, elevator));*/

    //driveController.rightTrigger().onTrue(new AutoAlignIntake(swerve, vision));
    //driveController.povUp().onTrue(new INPUTCLIMBCOMMANDUP));
    //driveController.povUpRight().onTrue(new INPUTCLIMBCOMMANDUP));
    //driveController.povUpLeft().onTrue(new INPUTCLIMBCOMMANDUP));
    //driveController.povDown().onTrue(new INPUTCLIMBCOMMANDDon));
    //driveController.povDownRight().onTrue(new INPUTCLIMBCOMMANDDon));
    //driveController.povDownLeft().onTrue(new INPUTCLIMBCOMMANDDon));

    //driveController.leftTrigger(.3).whileTrue(Commands.run(() -> swerve.driveWithAngleSetPoint(driveController.getLeftY(), driveController.getLeftX(), 30)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("test-path");
  }
}
