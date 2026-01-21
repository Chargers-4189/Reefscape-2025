// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCoral extends Command {
  private CoralEffector coraleffector;
  private int stage = 0;


  /** Creates a new OuttakeCoral. */
  public OuttakeCoral(CoralEffector coraleffector) {
    this.coraleffector = coraleffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coraleffector);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stage == 0){
      if(coraleffector.getAnalogSensor()){
        if(!coraleffector.getDigitalSensor()){
        coraleffector.moveEffector(); // going until first sensor the coral somes in contact with is false
        }else{
        stage = 1;
      }
      }
    }if(stage == 1){
      if(!coraleffector.getAnalogSensor()){
        coraleffector.moveEffectorBackwards(); //going backwards until the sensor closest to intake is true
      }else{
        stage = 3; // 3 = finished
      }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     coraleffector.stopEffector();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(stage == 3){
      return true;
    }else{
    return false;
    }
  }
}
