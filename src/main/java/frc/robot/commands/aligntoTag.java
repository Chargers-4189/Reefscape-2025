// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.util.Camera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aligntoTag extends Command {
  Vision vis = new Vision();
  Camera cam = new Camera();
  boolean direction;
  //direction = false move left true right

  //swerver may become a problem
  SwerveSubsystem swerve;
  /** Creates a new aligntoTag. */
  public aligntoTag(Vision vis, Camera cam, SwerveSubsystem swerve, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vis);
    this.swerve = swerve;
    this.vis = vis;
    this.cam = cam;
    this.direction = direction;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // swerve.drive(new Translation2d(-0.2 ,0.0),0.0,false);
   
   try{
    if(!direction){
      System.out.println("right side");
    PhotonTrackedTarget target = cam.getBestResultRightCam(vis).getBestTarget();
    double x = target.bestCameraToTarget.getX();
    double y = target.bestCameraToTarget.getY();
    System.out.println("x: " + x);
    System.out.println("Y: " + y);
if(x > .5){
    if(y > 0.10){
      swerve.drive(new Translation2d(0.25,0.25), 0, false);
    }else if(y < -0.10){
      swerve.drive(new Translation2d(0.25,-0.25), 0, false);
    }else{
      swerve.drive(new Translation2d(0.25,0), 0, false);
    }
  }else{
    if(y > 0.05){
      swerve.drive(new Translation2d(0,0.25), 0, false);
    }else if(y < -0.05){
      swerve.drive(new Translation2d(0,-0.25), 0, false);
    }else{
      end(true);
      isFinished();
    }
  }
}else{
  PhotonTrackedTarget target = cam.getBestResultleftCam(vis).getBestTarget();
    double x = target.bestCameraToTarget.getX();
    double y = target.bestCameraToTarget.getY();
    System.out.println("lift Side");
    System.out.println("x: " + x);
    System.out.println("Y: " + y);
if(x > .5){
    if(y > 0.09){
      swerve.drive(new Translation2d(0.25,-0.25), 0, false);
    }else if(y < -0.07){
      swerve.drive(new Translation2d(0.25,0.25), 0, false);
    }else{
      swerve.drive(new Translation2d(0.25,0), 0, false);
    }

    }else{
      if(y > 0.09){
      swerve.drive(new Translation2d(0,0.25), 0, false);
    }else if(y < -0.07){
      swerve.drive(new Translation2d(0,-0.25), 0, false);
    }else{
      if(target.getYaw() > 0.1 && target.getYaw() < -0.1){
        if(target.getYaw() > 0.1){
          swerve.drive(new Translation2d(0,0),Units.degreesToRadians(2), false);
        }else{
          swerve.drive(new Translation2d(0,0),Units.degreesToRadians(-2), false);
        }
      }else{
      end(true);
      isFinished();
      }
    }
  }
}

  }catch(Exception e){
    //System.out.println(e);
  }
    
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     swerve.drive(new Translation2d(0,0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

