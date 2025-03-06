// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.AUTO_LevelFour;
import frc.robot.commands.ActuateIntakeDown;
import frc.robot.commands.ActuateIntakeUp;
import frc.robot.commands.AlignReef;
import frc.robot.commands.AutoPlaceCoral;
import frc.robot.commands.CancelAll;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSlightlyDown;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.util.Elastic;
import frc.util.Elastic.ElasticClimber;

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
  private final Vision vision = new Vision();
  private final Climber climber = new Climber();

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

    final Trigger chuteTrigger = new Trigger(()->(
      (Math.abs(secondaryController.getLeftTriggerAxis()) > Constants.OperatorConstants.kSecondaryDeadband)
      || (Math.abs(secondaryController.getRightTriggerAxis()) > Constants.OperatorConstants.kSecondaryDeadband)
    ));

    final Trigger effectorTrigger = new Trigger(()->(Math.abs(secondaryController.getRightY()) > Constants.OperatorConstants.kSecondaryDeadband));

    coralEffector.setDefaultCommand(new IntakeCoral(coralEffector));

    //Driver Controls:

    swerve.setDefaultCommand(
      new TeleopDrive(swerve, driveController) //Movement including nitro
    );
    driveController.start().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(Commands.sequence(new MoveElevator( elevator, 0), new MoveElevatorSlightlyDown(elevator)));
    driveController.x().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 1));
    driveController.y().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 2));
    driveController.b().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator,coralEffector, 3));
    driveController.a().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 4));
    driveController.povDown().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new MoveElevator( elevator, 5));
    driveController.povUp().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new MoveElevator( elevator, 6));

    driveController.leftTrigger().whileTrue(new EjectAlgae(coralEffector));

    //driveController.leftBumper().onTrue(new AlignReef(swerve, vision, false).withTimeout(3));
    //driveController.rightBumper().onTrue(new AlignReef(swerve, vision, true).withTimeout(3));

    driveController
      .leftBumper()
      .onTrue(new AlignReef(swerve, vision, false).withTimeout(2));
    driveController
      .rightBumper()
      .onTrue(new AlignReef(swerve, vision, true).withTimeout(2));

    driveController.back().onTrue(new CancelAll(coralEffector, elevator, intake, swerve));

    driveController.rightTrigger().whileTrue(swerve.xFormation());
    
    /*
    driveController.start().debounce(1).onTrue(
        Commands.runOnce(
          () -> {
            swerve.zeroGyro();
          },
          swerve
        )
      );*/
    /*
    driveController.leftTrigger(.5).and(() -> coralEffector.state == "empty").onTrue(
      Commands.parallel(new EjectAlgae(coralEffector), new MoveElevator(elevator, 5))
    );
    driveController.leftTrigger(.5).and(() -> coralEffector.state == "empty").onFalse(
      Commands.parallel(
        new IntakeCoral(coralEffector),
        Commands.sequence(new MoveElevator(elevator, 0), new MoveElevatorSlightlyDown(elevator))
      )
    );

    driveController.rightTrigger(.5).and(() -> coralEffector.state == "empty").onTrue(
      Commands.parallel(new EjectAlgae(coralEffector), new MoveElevator(elevator, 6))
    );
    driveController.rightTrigger(.5).and(() -> coralEffector.state == "empty").onFalse(
      Commands.parallel(
        new IntakeCoral(coralEffector),
        Commands.sequence(new MoveElevator(elevator, 0), new MoveElevatorSlightlyDown(elevator))
      )
    );
    */

    //Secondary Controls:

    secondaryController.leftBumper().and(() -> !effectorTrigger.getAsBoolean()).onTrue(new ActuateIntakeDown(intake));
    secondaryController.rightBumper().and(() ->!effectorTrigger.getAsBoolean()).onTrue(new ActuateIntakeUp(intake));


    elevatorTrigger.whileTrue(Commands.run(()->{
      elevator.setVoltage(-secondaryController.getLeftY() * 4);
    },elevator));

    elevatorTrigger.onFalse(Commands.run(() -> {
      elevator.setVoltage(0);
    }, elevator));
    
    chuteTrigger.whileTrue(Commands.run(()-> {
      intake.setPower(secondaryController.getRightTriggerAxis() - secondaryController.getLeftTriggerAxis());
    },intake));

    chuteTrigger.onFalse(Commands.run(()-> {
      intake.setPower(0);
    },intake));

    effectorTrigger.whileTrue(Commands.run(()->{
      coralEffector.setPower(secondaryController.getRightY() * -.5);
    },coralEffector));

    effectorTrigger.onFalse(Commands.run(()->{
      coralEffector.setPower(0);
    },coralEffector));

    secondaryController.back().onTrue(new CancelAll(coralEffector, elevator, intake, swerve));


    secondaryController.x().onTrue(new MoveElevator( elevator, 1));
    secondaryController.y().onTrue(new MoveElevator( elevator, 2));
    secondaryController.b().onTrue(new MoveElevator( elevator,3));
    secondaryController.a().onTrue(new MoveElevator( elevator, 4));
    secondaryController.start().onTrue(Commands.sequence(new MoveElevator( elevator, 0), new MoveElevatorSlightlyDown(elevator)));

    secondaryController.povUp().onTrue(Commands.run(()->{
      //climber.setPower(0.5);
    }));
    secondaryController.povDown().onTrue(Commands.run(()->{
      //climber.setPower(-0.5);
    }));

    //Testing:

    /*
    driveController.povUp().onTrue(new MoveElevator(elevator, -1));

    driveController.povDown().onTrue(Commands.sequence(
      new MoveElevator(elevator, 0),
      new MoveElevatorSlightlyDown(elevator)
    ));*/


    //OLD:

    //driveController.leftTrigger(0.5).onTrue(new OuttakeCoral(coralEffector));
    //driveController.rightTrigger(.5).whileTrue(new ActuateIntakeDown(intake));

    /*
    elevator.setDefaultCommand(Commands.run(()->{
      elevator.setVoltage(-Math.pow(driveController.getRightY(), 3) * 1.5);
    }, elevator));*/

    //driveController.rightTrigger().onTrue(new AutoAlignIntake(swerve, vision));

    //driveController.leftTrigger(.3).whileTrue(Commands.run(() -> swerve.driveWithAngleSetPoint(driveController.getLeftY(), driveController.getLeftX(), 30)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new PathPlannerAuto("test-path");
    //return new ActuateIntakeUp(intake);
    return new AUTO_LevelFour(swerve, vision, elevator, coralEffector, intake, 4, true);

  }
}
