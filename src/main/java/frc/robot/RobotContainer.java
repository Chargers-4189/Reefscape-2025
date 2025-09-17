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
import frc.robot.commands.effector.IntakeCoral;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.MoveElevatorSlightlyDown;
import frc.robot.commands.intake.ActuateIntakeDown;
import frc.robot.commands.intake.ActuateIntakeUp;
import frc.robot.commands.multiaction.AlignAccuracyTest;
import frc.robot.commands.multiaction.CenterAuto;
import frc.robot.commands.multiaction.PlaceCoral;
import frc.robot.commands.multiaction.Taxi;
import frc.robot.commands.multiaction.ThreeCoralAuto;
import frc.robot.commands.multiaction.OneCoralAuto;
import frc.robot.commands.swervedrive.AlignReef;
import frc.robot.commands.swervedrive.Drive;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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
  private final CommandXboxController secondaryController = new CommandXboxController(
    1
  );

  // The robot's subsystems and commands are defined here...
  private final Elevator elevator = new Elevator();
  private final CoralEffector coralEffector = new CoralEffector();
  private final Intake intake = new Intake();
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
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoChooser();
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

    //Secondary Triggers
    final Trigger elevatorTrigger = new Trigger(() ->
      (
        Math.abs(secondaryController.getLeftY()) >
        Constants.OperatorConstants.kSecondaryDeadband
      )
    );

    final Trigger chuteTrigger = new Trigger(() ->
      (
        (
          Math.abs(secondaryController.getLeftTriggerAxis()) >
          Constants.OperatorConstants.kSecondaryDeadband
        ) ||
        (
          Math.abs(secondaryController.getRightTriggerAxis()) >
          Constants.OperatorConstants.kSecondaryDeadband
        )
      )
    );

    final Trigger effectorTrigger = new Trigger(() ->
      (
        Math.abs(secondaryController.getRightY()) >
        Constants.OperatorConstants.kSecondaryDeadband
      )
    );

    //Defaults
    coralEffector.setDefaultCommand(new IntakeCoral(coralEffector));

    //Primary

    //Driving
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

    //Elevator + Effector
    primaryController
      .povDown()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(
        Commands.sequence(
          new MoveElevator(elevator, 0),
          new MoveElevatorSlightlyDown(elevator)
        )
      );
    primaryController
      .x()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new PlaceCoral(elevator, coralEffector, 1));
    primaryController
      .y()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new PlaceCoral(elevator, coralEffector, 2));
    primaryController
      .b()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new PlaceCoral(elevator, coralEffector, 3));
    primaryController
      .a()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new PlaceCoral(elevator, coralEffector, 4));

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
        new CancelAll(coralEffector, elevator, intake, drivebase)
          .withTimeout(.5)
      );

    //Secondary

    //Cancel All
    secondaryController
      .back()
      .onTrue(
        new CancelAll(coralEffector, elevator, intake, drivebase)
          .withTimeout(.5)
      );

    //Intake
    secondaryController.leftBumper().onTrue(new ActuateIntakeDown(intake));
    secondaryController.rightBumper().onTrue(new ActuateIntakeUp(intake));

    chuteTrigger.whileTrue(
      Commands.run(
        () -> {
          intake.setPower(
            secondaryController.getRightTriggerAxis() -
            secondaryController.getLeftTriggerAxis()
          );
        },
        intake
      )
    );

    chuteTrigger.onFalse(
      Commands.runOnce(
        () -> {
          intake.setPower(0);
        },
        intake
      )
    );

    //Effector
    effectorTrigger.whileTrue(
      Commands.run(
        () -> {
          coralEffector.setPower(secondaryController.getRightY() * -.5);
        },
        coralEffector
      )
    );

    effectorTrigger.onFalse(
      Commands.runOnce(
        () -> {
          coralEffector.setPower(0);
        },
        coralEffector
      )
    );

    //Elevator
    secondaryController.x().onTrue(new MoveElevator(elevator, 1));
    secondaryController.y().onTrue(new MoveElevator(elevator, 2));
    secondaryController.b().onTrue(new MoveElevator(elevator, 3));
    secondaryController.a().onTrue(new MoveElevator(elevator, 4));
    secondaryController
      .start()
      .onTrue(
        Commands.sequence(
          new MoveElevator(elevator, 0),
          new MoveElevatorSlightlyDown(elevator)
        )
      );

    elevatorTrigger.whileTrue(
      Commands.run(
        () -> {
          elevator.setVoltage(-secondaryController.getLeftY() * 4);
        },
        elevator
      )
    );

    elevatorTrigger.onFalse(
      Commands.runOnce(
        () -> {
          elevator.setVoltage(0);
        },
        elevator
      )
    );
    //secondaryController.povUp().whileTrue(
    //  Commands.parallel(
    //  new TwoCoralAuto(drivebase, elevator, coralEffector, false, true),
    //  new ActuateIntakeUp(intake)
    //)
    //);

    // secondaryController.povUp().whileTrue(Commands.run(()->{
    //   coralEffector.setPower(Constants.CoralEffectorConstants.kSECONDARY_OUT_POWER);
    // }));
    // secondaryController.povDown().whileTrue(Commands.run(()->{
    //   coralEffector.setPower(Constants.CoralEffectorConstants.kSECONDARY_IN_POWER);
    // }));

    //climber.setDefaultCommand(Commands.run(() -> climber.setPower(secondaryController.getRightY()), climber));
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("3 Coral Right", new ThreeCoralAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      true,
      drivebase.isRedAlliance()
    ));
    autoChooser.addOption("3 Coral Left", new ThreeCoralAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      false,
      drivebase.isRedAlliance()
    ));
    autoChooser.addOption("2 Coral Right", new OneCoralAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      true,
      drivebase.isRedAlliance()
    ));
    autoChooser.addOption("2 Coral Left",  new OneCoralAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      false,
      drivebase.isRedAlliance()
    ));
    autoChooser.setDefaultOption("1 Coral Right", new ThreeCoralAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      true,
      drivebase.isRedAlliance()
    ));
    autoChooser.addOption("1 Coral Center - Place Right",  new CenterAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      drivebase.isRedAlliance(),
      true
    ));
    autoChooser.addOption("1 Coral Center - Place Left",  new CenterAuto(
      drivebase,
      elevator,
      coralEffector,
      intake,
      drivebase.isRedAlliance(),
      false
    ));
    autoChooser.addOption("Taxi",  new Taxi(
      drivebase,
      coralEffector,
      intake
    ));
    autoChooser.addOption("Accuracy Test",  new AlignAccuracyTest(
      drivebase,
      elevator,
      coralEffector,
      intake
    ));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    return autoChooser.getSelected();
    /*
    return Commands.parallel(
      new TwoCoralAuto(
        drivebase,
        elevator,
        coralEffector,
        false,
        drivebase.isRedAlliance()
      ),
      new ActuateIntakeUp(intake)
    );*/
    /*
    return Commands.parallel(
      new PlaceThenGetCoral(
        drivebase,
        elevator,
        coralEffector,
        20,
        13,
        true
      ),
      new ActuateIntakeUp(intake)
    );*/
    //return new AprilTagPathPlannerAuto(drivebase, elevator, 19, false, 4);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
