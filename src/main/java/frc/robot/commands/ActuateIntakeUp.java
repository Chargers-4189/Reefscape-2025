// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.util.Networker.NetworkIntake;
import frc.util.Stopwatch;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActuateIntakeUp extends Command {

  private final Intake intake;
  private final Stopwatch stopwatch = new Stopwatch();


  /** Creates a new ActuateIntakeUp. */
  public ActuateIntakeUp(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init");
    stopwatch.start(2500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing");
    intake.setPower(NetworkIntake.kPOWER.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished");
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("switch: " + intake.getTopLimitSwitch() + "  stopwatch: " + stopwatch.hasTriggered());
    return (intake.getTopLimitSwitch()) || stopwatch.hasTriggered();
  }
}
