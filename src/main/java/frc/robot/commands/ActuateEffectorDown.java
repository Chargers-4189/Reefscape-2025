// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.robot.Constants.CoralEffectorConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActuateEffectorDown extends Command {
    private CoralEffector coralEffector;
  private double encoderValue;
  /** Creates a new ActuateEffectorDown. */
  public ActuateEffectorDown(CoralEffector coralEffector) {
    this.coralEffector = coralEffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderValue = coralEffector.getAbsoluteEncoderValue();
    //FIND GOAL VALUE
    if(encoderValue > CoralEffectorConstants.kDEFAULT_CORAL_EFFECTER_ANGLE){
      coralEffector.ActuateBackward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.StopActuating();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(encoderValue >= CoralEffectorConstants.kDEFAULT_CORAL_EFFECTER_ANGLE);
  }
}
