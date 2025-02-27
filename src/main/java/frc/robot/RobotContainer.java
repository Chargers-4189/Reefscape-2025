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
import frc.robot.commands.AlignReef;
import frc.robot.commands.AutoAlignPose;
import frc.robot.commands.AutoPlaceCoral;
import frc.robot.commands.CancelAll;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
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
  private final Vision vision = new Vision();

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

    driveController.leftTrigger(.8).onTrue(new ActuateIntakeDown(intake));
    driveController.rightTrigger(.8).onTrue(new ActuateIntakeUp(intake));

    driveController
      .leftBumper()
      .onTrue(new AlignReef(swerve, vision, false).withTimeout(6));
    driveController
      .rightBumper()
      .onTrue(new AlignReef(swerve, vision, true).withTimeout(6));

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

    driveController.x().onTrue(new AutoPlaceCoral(elevator, coralEffector, 1));
    driveController.y().onTrue(new AutoPlaceCoral(elevator, coralEffector, 2));
    driveController.b().onTrue(new AutoPlaceCoral(elevator, coralEffector, 3));
    driveController.a().onTrue(new AutoPlaceCoral(elevator, coralEffector, 4));
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
