// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActuateIntakeDown;
import frc.robot.commands.ActuateIntakeUp;
import frc.robot.commands.AlignReefAngle;
import frc.robot.commands.AlignReefPosition;
import frc.robot.commands.CancelAll;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSlightlyDown;
import frc.robot.commands.multiaction.AutoPlaceCoral;
import frc.robot.commands.multiaction.ThreeCoralAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

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
  private final Climber climber = new Climber();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream
    .of(
      drivebase.getSwerveDrive(),
      () -> primaryController.getLeftY() * -1,
      () -> primaryController.getLeftX() * -1
    )
    .withControllerRotationAxis(() -> -primaryController.getRightX())
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveWithStationAngle = driveAngularVelocity
    .copy()
    .withControllerHeadingAxis(
      () -> drivebase.getStationRotation().getCos(),
      () -> drivebase.getStationRotation().getSin()
    )
    .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    //autoChooser.setDefaultOption("17", drivebase.driveToAprilTag(17));
    //autoChooser.addOption("18", drivebase.driveToAprilTag(18));
    //autoChooser.addOption("19", drivebase.driveToAprilTag(19));
    //autoChooser.addOption("20", drivebase.driveToAprilTag(20));
    //autoChooser.addOption("21", drivebase.driveToAprilTag(21));
    //autoChooser.addOption("22", drivebase.driveToAprilTag(22));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
      driveAngularVelocity
    );
    Command driveFieldOrientedWithStationAngle = drivebase.driveFieldOriented(
      driveWithStationAngle
    );

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
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    primaryController
      .leftTrigger(.5)
      .whileTrue(driveFieldOrientedWithStationAngle);
    primaryController
      .rightTrigger(.5)
      .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
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
      .onTrue(new AutoPlaceCoral(elevator, coralEffector, 1));
    primaryController
      .y()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new AutoPlaceCoral(elevator, coralEffector, 2));
    primaryController
      .b()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new AutoPlaceCoral(elevator, coralEffector, 3));
    primaryController
      .a()
      .and(() -> !elevatorTrigger.getAsBoolean())
      .onTrue(new AutoPlaceCoral(elevator, coralEffector, 4));

    primaryController
      .leftBumper()
      .onTrue(Commands.sequence(
        new AlignReefAngle(drivebase).withTimeout(.4),
        new AlignReefPosition(drivebase, false).withTimeout(2.5))
      );
      primaryController
      .rightBumper()
      .onTrue(Commands.sequence(
        new AlignReefAngle(drivebase).withTimeout(.4),
        new AlignReefPosition(drivebase, true).withTimeout(2.5))
      );

    
    primaryController.povUp().onTrue(Commands.run(() -> drivebase.driveToReefClosest(false).withTimeout(.5).schedule(), drivebase));
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
    //  new ThreeCoralAuto(drivebase, elevator, coralEffector, false, true),
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    return Commands.parallel(
      new ThreeCoralAuto(
        drivebase,
        elevator,
        coralEffector,
        false,
        drivebase.isRedAlliance()
      ),
      new ActuateIntakeUp(intake)
    );
    //return new AprilTagPathPlannerAuto(drivebase, elevator, 19, false, 4);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
