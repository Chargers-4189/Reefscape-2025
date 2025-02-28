// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.util.Stopwatch;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private CoralEffector coralEffector;
  private Stopwatch stopwatch = new Stopwatch();

  /**
   * 
   *  If intake sensor is active, inputs coral until outtake sensor sees coral
   * @param coralEffector import coral subsystem
   */
  public IntakeCoral(CoralEffector coralEffector) {
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
    /*
    if (!coralEffector.getIntakeSensor() && !coralPassed && !coralEntered) {
      coralEffector.setPower(0.2);
    } else if (coralEffector.getIntakeSensor() && !coralPassed) {
      coralEffector.setPower(0.1);
      coralEntered = true;
    } else if (!coralEffector.getIntakeSensor()) {
      coralEffector.setPower(-0.05);
      coralPassed = true;
    } else if (coralEffector.getIntakeSensor()) {
      coralEffector.stop();
      coralSecured = true;
    }*/
    /*switch(coralEffector.state) {
      case "empty":
        if (coralEffector.getIntakeSensor()) {
          coralEffector.state = "pull_in";
        }
        return;
      case "pull_in":
        coralEffector.setPower(0.1);
        if (!coralEffector.getIntakeSensor()) {
          coralEffector.state = "back_up_1";
        }
        return;
      case "back_up_1": 
        coralEffector.setPower(-0.05);
        if (coralEffector.getIntakeSensor()) {
          stopwatch.start(160);
          coralEffector.state = "back_up_2";
        }
        return;
      case "back_up_2":
        coralEffector.setPower(-0.05);
        if (stopwatch.hasTriggered()) {
          coralEffector.state = "done";
        }
        return;
      case "done":
      coralEffector.stop();
        if (!coralEffector.getIntakeSensor()) {
          coralEffector.state = "empty";
        }
        return;
      default:
        System.out.println("ERROR: Invalid Coral Intake State");
    } **/
   if(coralEffector.getIntakeSensor() == true){
    coralEffector.setPower(-0.2);
   }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralEffector.getOuttakeSensor();
  }
}
