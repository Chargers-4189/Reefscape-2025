// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {

  private final Elevator elevatorSubsystem;
  private final PIDController feedback = new PIDController(0.0, 0.0, 0.0);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
  private final TrapezoidProfile trapezoidProfiler = new TrapezoidProfile(new Constraints(.3, .3));
  private int level;
  private State startState;
  private State goalState;
  private State setpoint;
  private double startTime;
  private double feedbackVoltage;
  private double feedforwardVoltage;


  /**
   * Creates a new moveElevator command.
   *
   * @param level The level to move the elevator to. 0 moves to the intake.
   */
  public MoveElevator(Elevator elevatorSubsystem, int level) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.level = level;

    this.startTime = Timer.getFPGATimestamp();
    this.startState = new State(elevatorSubsystem.getHeightMeters(), elevatorSubsystem.getVelocityMeters());
    this.goalState = new State(ElevatorConstants.HEIGHTS_METERS[level], 0);

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
      feedback.reset();
    }

    //currentState = new State(elevatorSubsystem.getHeightMeters(), elevatorSubsystem.getVelocityMeters());

    setpoint = trapezoidProfiler.calculate(Timer.getFPGATimestamp() - startTime, startState, goalState);

    feedbackVoltage = feedback.calculate(elevatorSubsystem.getHeightMeters(), setpoint.position);
    feedforwardVoltage = feedforward.calculate(setpoint.velocity);
    

    //elevatorSubsystem.setVoltage(feedbackVoltage + feedforwardVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setLevel(level);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapezoidProfiler.isFinished(Timer.getFPGATimestamp() - startTime);
  }
}
