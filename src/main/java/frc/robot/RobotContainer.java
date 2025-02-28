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
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSlightlyDown;
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
  private final CommandXboxController secondaryController = new CommandXboxController(
    Constants.OperatorConstants.secondaryController
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
    //OperatorControl Triggers (activated when operator controls are used)

    final Trigger elevatorTrigger = new Trigger(()->(Math.abs(secondaryController.getLeftY()) > Constants.OperatorConstants.kSecondaryDeadband)); 

    final Trigger effectorTrigger = new Trigger(()->(
      (Math.abs(secondaryController.getLeftTriggerAxis()) > Constants.OperatorConstants.kSecondaryDeadband)
      || (Math.abs(secondaryController.getRightTriggerAxis()) > Constants.OperatorConstants.kSecondaryDeadband)
    ));

    final Trigger chuteTrigger = new Trigger(()->(Math.abs(secondaryController.getRightY()) > Constants.OperatorConstants.kSecondaryDeadband));

    //Default Commands:

    coralEffector.setDefaultCommand(new IntakeCoral(coralEffector));

    swerve.setDefaultCommand(
      swerve.driveCommand(
        () -> Math.pow(-driveController.getLeftY(), Constants.HumanDriveConstants.kDRIVE_EXPONENT)
        * Constants.HumanDriveConstants.kDRIVE_POWER,
        () -> Math.pow(-driveController.getLeftX(), Constants.HumanDriveConstants.kDRIVE_EXPONENT)
        * Constants.HumanDriveConstants.kDRIVE_POWER,
        () -> Math.pow(-driveController.getRightX(), Constants.HumanDriveConstants.kROTATIONAL_EXPONENT)
        * Constants.HumanDriveConstants.kROTATIONAL_POWER,
        true
      )
    );
    

    //Driver Controls:

    driveController.x().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 1));
    driveController.y().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 2));
    driveController.b().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator,coralEffector, 3));
    driveController.a().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 4));

    //driveController.leftBumper().onTrue(new AlignReef(swerve, vision, false).withTimeout(3));
    //driveController.rightBumper().onTrue(new AlignReef(swerve, vision, true).withTimeout(3));

    driveController.back().onTrue(new CancelAll(coralEffector, elevator, intake, swerve));

    driveController.start().debounce(1).onTrue(
        Commands.runOnce(
          () -> {
            swerve.zeroGyro();
          },
          swerve
        )
      );

    driveController.leftTrigger(.5).and(()  -> !elevatorTrigger.getAsBoolean() && coralEffector.state == "empty").onTrue(
      Commands.parallel(new EjectAlgae(coralEffector), new MoveElevator(elevator, 5))
    ).onFalse(
      Commands.parallel(
        new IntakeCoral(coralEffector),
        Commands.sequence(new MoveElevator(elevator, 0), new MoveElevatorSlightlyDown(elevator))
      )
    );

    driveController.rightTrigger(.5).and(()  -> !elevatorTrigger.getAsBoolean() && coralEffector.state == "empty").onTrue(
      Commands.parallel(new EjectAlgae(coralEffector), new MoveElevator(elevator, 6))
    ).onFalse(
      Commands.parallel(
        new IntakeCoral(coralEffector),
        Commands.sequence(new MoveElevator(elevator, 0), new MoveElevatorSlightlyDown(elevator))
      )
    );

    //Secondary Controls:

    secondaryController.leftBumper().and(() -> !effectorTrigger.getAsBoolean()).onTrue(new ActuateIntakeDown(intake));
    secondaryController.rightBumper().and(() ->!effectorTrigger.getAsBoolean()).onTrue(new ActuateIntakeUp(intake));


    elevatorTrigger.whileTrue(Commands.run(()->{
      elevator.setVoltage(-secondaryController.getLeftY() * 4);
    },elevator));
    
    effectorTrigger.whileTrue(Commands.run(()-> {
      coralEffector.setPower(secondaryController.getRightTriggerAxis() - secondaryController.getLeftTriggerAxis());
    },coralEffector));

    chuteTrigger.whileTrue(Commands.run(()->{
      intake.setPower(secondaryController.getRightY());
    },intake));

    secondaryController.back().onTrue(new CancelAll(coralEffector, elevator, intake, swerve));

    //Testing:

    driveController.povUp().onTrue(new MoveElevator(elevator, -1));
    driveController.povDown().onTrue(Commands.sequence(
      new MoveElevator(elevator, 0),
      new MoveElevatorSlightlyDown(elevator)
    ));


    //OLD:

    //driveController.leftTrigger(0.5).onTrue(new OuttakeCoral(coralEffector));
    //driveController.rightTrigger(.5).whileTrue(new ActuateIntakeDown(intake));

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
