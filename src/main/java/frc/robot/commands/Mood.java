// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import com.fasterxml.jackson.databind.ObjectMapper;
//import java.io.File;
//import java.util.ArrayList;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map;

//import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Faces;
//import frc.util.Elastic.kHEIGHTS;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Mood extends Command {
  private Faces faces;
  private int mood;
  private int mode;
  private int frameNumber;
  private double faceTime;
  
  CommandXboxController controller;
  /** Creates a new smile. */
  public Mood(Faces faces, CommandXboxController controller) {
    this.faces = faces;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(faces);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mode = 0;
    mood = -1;
    faces.clear();
    faces.resetTime();
    faces.startTime();
    faceTime = 0;
    frameNumber = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("FACE EXECUTING");
    //smile (Default)
    if (controller.povLeft().getAsBoolean()) {
      mood = 0;
    }
    //dizzy (Lost Comms)
    else if (controller.povUp().getAsBoolean()) {
      mood = 1;
    }
    //submerge (With Elevator)
    else if (controller.povRight().getAsBoolean()) {
      mood = 2;
    }
    else if (controller.povDown().getAsBoolean()) {
      mood = -1;
    }

    switch (mood) {
      case -1:
        faces.smile();
        break;
      case 0:
        faces.angry(3);
        break;
      case 1:
        faces.pirate();
        break;
      case 2:
        faceTime = (faces.getTime() - (0.5625 * frameNumber));
        if(faceTime >= 0.5625){
          mode++;
          mode = (mode % 2); //change frame number here
          frameNumber++;       
        }           
        faces.party(mode); //change face frames here
        break;
      }
    }


    //This is example code for a 4 frame animation in case I may lose it
    /*
       faceTime = (faces.getTime() - (0.5625 * frameNumber));
      if(faceTime >= 0.5625){
        mode++;
        mode = (mode % 4); //change frame number here
        frameNumber++;       
      }           
      faces.jeremy(mode); //change face frames here
      break;
     */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}