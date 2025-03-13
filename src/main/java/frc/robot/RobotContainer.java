// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActuateIntakeDown;
import frc.robot.commands.ActuateIntakeUp;
import frc.robot.commands.AprilTagPathPlannerAuto;
import frc.robot.commands.CancelAll;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSlightlyDown;
import frc.robot.commands.multiaction.ThreeCoralAuto;
import frc.robot.commands.multiaction.AutoPlaceCoral;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final Elevator elevator = new Elevator();
  private final CoralEffector coralEffector = new CoralEffector();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driveController.getLeftY() * -1,
                                                                () -> driveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driveController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
  public RobotContainer()
  {

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
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity); 
    /*
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);*/

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    /*
    if (Robot.isSimulation())
    {
      driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
                                                            Rotation2d.fromDegrees(90)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5,
                                                                                     3)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(
                                                                         Math.toRadians(
                                                                             360),
                                                                         Math.toRadians(
                                                                             90))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }*/

    //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    driveController.rightTrigger(.5).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    /*
    driverXbox.povDown().onTrue(new AprilTagPathPlannerAuto(drivebase, elevator, 18, true, 4));
    driverXbox.povDownLeft().onTrue(new AprilTagPathPlannerAuto(drivebase, elevator, 19, true, 4));
    driverXbox.povDownRight().onTrue(new AprilTagPathPlannerAuto(drivebase, elevator, 17, true, 4));
    driverXbox.povUpRight().onTrue(new AprilTagPathPlannerAuto(drivebase, elevator, 22, true, 4));
    driverXbox.povUpLeft().onTrue(new AprilTagPathPlannerAuto(drivebase, elevator, 20, true, 4));
    driverXbox.povUp().toggleOnTrue(new AprilTagPathPlannerAuto(drivebase, elevator, 21, true, 4));*/

    final Trigger elevatorTrigger = new Trigger(()->(Math.abs(secondaryController.getLeftY()) > .1)); 

    final Trigger chuteTrigger = new Trigger(()->(
      (Math.abs(secondaryController.getLeftTriggerAxis()) > Constants.OperatorConstants.kSecondaryDeadband)
      || (Math.abs(secondaryController.getRightTriggerAxis()) > Constants.OperatorConstants.kSecondaryDeadband)
    ));

    final Trigger effectorTrigger = new Trigger(()->(secondaryController.povUp().getAsBoolean() || secondaryController.povDown().getAsBoolean()));

    coralEffector.setDefaultCommand(new IntakeCoral(coralEffector));

    //Driver Controls:

    driveController.start().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(Commands.sequence(new MoveElevator( elevator, 0), new MoveElevatorSlightlyDown(elevator)));
    driveController.x().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 1));
    driveController.y().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 2));
    driveController.b().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator,coralEffector, 3));
    driveController.a().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new AutoPlaceCoral( elevator, coralEffector, 4));
    driveController.povDown().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new MoveElevator( elevator, 5));
    driveController.povUp().and(() -> !elevatorTrigger.getAsBoolean()).onTrue(new MoveElevator( elevator, 6));

    driveController.leftTrigger().whileTrue(new EjectAlgae(coralEffector));

    driveController.leftBumper().onTrue(Commands.run(() -> drivebase.driveToReefClosest(false).withTimeout(3).schedule(), drivebase));
    driveController.rightBumper().onTrue(Commands.run(() -> drivebase.driveToReefClosest(true).withTimeout(3).schedule(), drivebase));

    
    
    
    //driveController.leftBumper().onTrue(new AlignReefAngle(swerve, vision, true));

    driveController.back().onTrue(new CancelAll(coralEffector, elevator, intake, drivebase));
    
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

    secondaryController.leftBumper().onTrue(new ActuateIntakeDown(intake));
    secondaryController.rightBumper().onTrue(new ActuateIntakeUp(intake));
    
    chuteTrigger.whileTrue(Commands.run(()-> {
      intake.setPower(secondaryController.getRightTriggerAxis() - secondaryController.getLeftTriggerAxis());
    },intake));

    chuteTrigger.onFalse(Commands.runOnce(()-> {
      intake.setPower(0);
    },intake));
    


    elevatorTrigger.whileTrue(Commands.run(()->{
      elevator.setVoltage(-secondaryController.getLeftY() * 4);
    },elevator));

    elevatorTrigger.onFalse(Commands.runOnce(() -> {
      elevator.setVoltage(0);
    }, elevator));

    effectorTrigger.whileTrue(Commands.run(()->{
      coralEffector.setPower(secondaryController.getRightY() * -.5);
    },coralEffector));

    effectorTrigger.onFalse(Commands.runOnce(()->{
      coralEffector.setPower(0);
    },coralEffector));

    secondaryController.back().onTrue(new CancelAll(coralEffector, elevator, intake, drivebase));


    secondaryController.x().onTrue(new MoveElevator( elevator, 1));
    secondaryController.y().onTrue(new MoveElevator( elevator, 2));
    secondaryController.b().onTrue(new MoveElevator( elevator,3));
    secondaryController.a().onTrue(new MoveElevator( elevator, 4));
    secondaryController.start().onTrue(Commands.sequence(new MoveElevator( elevator, 0), new MoveElevatorSlightlyDown(elevator)));

    secondaryController.povUp().whileTrue(Commands.run(()->{
      coralEffector.setPower(Constants.CoralEffectorConstants.kSECONDARY_OUT_POWER);
    }));
    secondaryController.povDown().whileTrue(Commands.run(()->{
      coralEffector.setPower(Constants.CoralEffectorConstants.kSECONDARY_IN_POWER);
    }));

    climber.setDefaultCommand(Commands.run(() -> climber.setPower(secondaryController.getRightY()), climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    return Commands.parallel(
      new ThreeCoralAuto(drivebase, elevator, coralEffector),
      new ActuateIntakeUp(intake)
    );
    //return new AprilTagPathPlannerAuto(drivebase, elevator, 19, false, 4);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
