// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorController extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final PIDController elevatorFeedback = new PIDController(0.0, 0.0, 0.0);
  private double setpoint;
  private int level;


  /** Creates a new MoveElevator. */
  public ElevatorController(ElevatorSubsystem elevatorSubsystem, int level) {
    this.elevatorSubsystem = elevatorSubsystem;

    this.level = level;
    this.setpoint = ElevatorConstants.HEIGHTS_METERS[level];

    elevatorFeedback.setTolerance(ElevatorConstants.kTOLERANCE_METERS);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {};

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorSubsystem.getMinLimitSwitch()) {
      elevatorSubsystem.zeroEncoder();
      elevatorFeedback.reset();
    }

    elevatorSubsystem.setPower(elevatorFeedback.calculate(elevatorSubsystem.getHeightMeters(), setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setLevel(level);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorFeedback.atSetpoint();
  }
}
