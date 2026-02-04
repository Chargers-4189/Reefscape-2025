// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private CoralEffector coraleffector;
  private int stage = 0;


  /** Creates a new IntakeCoral. */
  public IntakeCoral(CoralEffector coraleffector) {
    this.coraleffector = coraleffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coraleffector);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = -1;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //System.out.println(stage);
  //System.out.println("Digital: "  + coraleffector.getDigitalSensor());
  //System.out.println("Analog: "  + coraleffector.getAnalogSensor());
    if(coraleffector.getDigitalSensor() && !coraleffector.getAnalogSensor()){
      stage = 0;
    }
    if(stage == 0){
      if(coraleffector.getDigitalSensor()){
        if(!coraleffector.getAnalogSensor()){
          //System.out.println("Stage 0");
        coraleffector.moveEffector(); // going until first sensor the coral comes in contact with is false
        }else{
        stage = 1;
      }
      }
    }

    if(stage == 1){
      if(coraleffector.getDigitalSensor()){
        //System.out.println("Stage 1");
        coraleffector.moveEffector();
      }else{
        stage = 2; // 3 = finished
      }
    }
    if(stage == 2){
      if(!coraleffector.getDigitalSensor()){
        //System.out.println("Stage 2");
        coraleffector.moveEffectorBackwards(); //going backwards until the sensor closest to intake is true
      }else{
        //System.out.println("killmenow");
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
    if(stage == 3 || (stage == -1 && coraleffector.getDigitalSensor() && coraleffector.getAnalogSensor())){
      //System.out.println("Kill me again");
      return true;
    }else{
    return false;
    }
  }
}
