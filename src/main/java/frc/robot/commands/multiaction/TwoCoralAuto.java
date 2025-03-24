// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multiaction;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.effector.IntakeCoral;
import frc.robot.commands.effector.OuttakeCoral;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.MoveElevatorSlightlyDown;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.swervedrive.AlignReef;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCoralAuto extends SequentialCommandGroup {
  private int reefId1;
  private int reefId2;
  private int stationId;
  /** Creates a new ThreeCoralAuto. */
  public TwoCoralAuto(SwerveSubsystem swerve, Elevator elevator, CoralEffector effector, boolean rightStart, boolean red) {


    if (red) {
      if (rightStart) {
        //red-right
        reefId1 = 9;
        reefId2 = 8;
        stationId = 2;
      } else {
        //red-left
        reefId1 = 11;
        reefId2 = 6;
        stationId = 1;
      }
    } else {
      if (rightStart) {
        //blue-right
        reefId1 = 20;
        reefId2 = 17;
        stationId = 22;
      } else {
        //blue-left
        reefId1 = 20;
        reefId2 = 19;
        stationId = 13;
      }
    }
    
    

    
    // Add your commands in the addCommands() call.
    addCommands(
      //Go to Reef
      //swerveSubsystem.driveToAprilTag(20, new Translation2d(1,0)),
      //Place Coral
      new AlignReef(swerve, true, reefId1),

      new MoveElevator(elevator, 4),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      //Go to Station
      Commands.parallel(
        new MoveElevatorSlightlyDown(elevator),
        swerve.driveToAprilTag(stationId, 0)
      ),
      //Wait, then go to Reef
      Commands.race(
        new IntakeCoral(effector),
        Commands.sequence(
          Commands.waitTime(Time.ofBaseUnits(1, Seconds)),
          new AlignReef(swerve, false, reefId2)
        )
      ),
      //Place Coral
      new MoveElevator(elevator, 4),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      //Go to Station
      Commands.parallel(
        new MoveElevatorSlightlyDown(elevator),
        swerve.driveToAprilTag(stationId, 0)
      )

      /*
      //Wait, then go to Reef
      Commands.race(
        new IntakeCoral(effector),
        Commands.sequence(
          Commands.waitTime(Time.ofBaseUnits(1, Seconds)),
          swerveSubsystem.driveToReef(19, true)
        )
      ),
      //Place Coral
      new MoveElevator(elevator, 4),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0)
      */
      /*
       * //Place Coral
      Commands.parallel(
        swerveSubsystem.driveToReef(20, true),
        new MoveElevator(elevator, 4)
      ),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      //Get Coral, then return to reef
      Commands.race(
        new MoveElevatorSlightlyDown(elevator),
        new IntakeCoral(effector),
        Commands.sequence(
          swerveSubsystem.driveToAprilTag(13),
          Commands.waitTime(Time.ofBaseUnits(.6, Seconds)),
          swerveSubsystem.driveToAprilTag(20, new Translation2d(1,0))
        )
      )
      /*
       */
      /*
      swerveSubsystem.driveToAprilTag(20, new Translation2d(1.8,0)),
      swerveSubsystem.driveToReef(20, true),
      new MoveElevator(elevator, 4),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      new DriveToCoralStation(swerveSubsystem, elevator, effector),
      Commands.waitTime(Time.ofBaseUnits(1, Seconds)),
      //swerveSubsystem.driveToAprilTag(13),
      //swerveSubsystem.driveToAprilTag(19, new Translation2d(1,0)),
      //swerveSubsystem.driveToReef(19, false),
      //swerveSubsystem.driveToAprilTag(13),
      //swerveSubsystem.driveToAprilTag(19, new Translation2d(1,0)),
      //swerveSubsystem.driveToReef(19, true)
      swerveSubsystem.driveToAprilTag(19, new Translation2d(1.5,0)),
      swerveSubsystem.driveToReef(19, false),
      new MoveElevator(elevator, 4),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      new DriveToCoralStation(swerveSubsystem, elevator, effector),
      Commands.waitTime(Time.ofBaseUnits(1, Seconds)),
      
      swerveSubsystem.driveToAprilTag(19, new Translation2d(1.5,0)),
      swerveSubsystem.driveToReef(19, true),
      new MoveElevator(elevator, 4),
      new OuttakeCoral(effector),
      new MoveElevator(elevator, 0),
      new DriveToCoralStation(swerveSubsystem, elevator, effector),
      Commands.waitTime(Time.ofBaseUnits(1, Seconds)),
      
      //addCommands(
      //Commands.race(
      //  new IntakeCoral(effector),
      //  Commands.sequence(
      //    swerveSubsystem.driveToAprilTag(13,0),
      //    Commands.waitTime(Time.ofBaseUnits(1, Seconds)),
      //    swerveSubsystem.driveToAprilTag(19, new Translation2d(1.5,0)),
      //  )
      //)//,
      //new AutoPlaceCoral(elevator, effector, reefLevel)
    //),

      new GetThenPlaceCoral(swerveSubsystem, elevator, effector, 13, 19, true, 4)
      */
    );
  }
}
