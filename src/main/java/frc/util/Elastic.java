// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;

/** Add your docs here. */
public class Elastic {

    static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    
    public static final class ElasticElevator {
        static NetworkTable elevatorTable = networkInstance.getTable("elevatorConstants");

        public static DoubleEntry kGRAVITY_VOLTS =
        elevatorTable
            .getDoubleTopic("GRAVITY_VOLTS")
            .getEntry(Constants.ElevatorConstants.kGRAVITY_VOLTS);
        public static DoubleEntry kMAX_VOLTS =
        elevatorTable
            .getDoubleTopic("MAX_VOLTS")
            .getEntry(Constants.ElevatorConstants.kMAX_VOLTS);
        public static DoubleEntry kMAX_VOLT_CHANGE_PER_SECOND =
        elevatorTable
            .getDoubleTopic("MAX_VOLT_CHANGE_PER_SECOND")
            .getEntry(Constants.ElevatorConstants.kMAX_VOLT_CHANGE_PER_SECOND);
        public static DoubleEntry kPROPORTIONAL_VOLTS =
        elevatorTable
            .getDoubleTopic("PROPORTIONAL_VOLTS")
            .getEntry(Constants.ElevatorConstants.kPROPORTIONAL_VOLTS);
        public static DoubleEntry kTOLERANCE =
        elevatorTable
            .getDoubleTopic("TOLERANCE")
            .getEntry(Constants.ElevatorConstants.kTOLERANCE);
        public static DoubleArrayEntry kHEIGHTS =
        elevatorTable
            .getDoubleArrayTopic("HEIGHTS")
            .getEntry(Constants.ElevatorConstants.kHEIGHTS);
        public static DoubleEntry kTEST_HEIGHT =
        elevatorTable
            .getDoubleTopic("TEST_HEIGHT")
            .getEntry(0);
        public static IntegerEntry kDOWN_TIMEOUT =
        elevatorTable
            .getIntegerTopic("DOWN_TIMEOUT")
            .getEntry(Constants.ElevatorConstants.kDOWN_TIMEOUT);

        public static void initialize() {
            kGRAVITY_VOLTS.set(kGRAVITY_VOLTS.get());
            kMAX_VOLTS.set(kMAX_VOLTS.get());
            kMAX_VOLT_CHANGE_PER_SECOND.set(kMAX_VOLT_CHANGE_PER_SECOND.get());
            kPROPORTIONAL_VOLTS.set(kPROPORTIONAL_VOLTS.get());
            kTOLERANCE.set(kTOLERANCE.get());
            kHEIGHTS.set(kHEIGHTS.get());
            kTEST_HEIGHT.set(kTEST_HEIGHT.get());
            kDOWN_TIMEOUT.set(kDOWN_TIMEOUT.get());
        }
    }
    public static final class ElasticSwerve {
        static NetworkTable swerveTable = networkInstance.getTable("swerveTable");

        public static StructPublisher<Pose2d> kROBOT_POSITION =
            swerveTable
                .getStructTopic("ROBOT_POSITION", Pose2d.struct).publish();
        public static StructPublisher<Pose2d> kGOAL_POSITION =
            swerveTable
                .getStructTopic("GOAL_POSITION", Pose2d.struct).publish();
        
        public static void setrobotPose(Pose2d robotPose) {
            kROBOT_POSITION.set(robotPose);
        }
        public static void setGoalPose(Pose2d goalPose) {
            kGOAL_POSITION.set(goalPose);
        }

        public static void initialize() {}
    }

    public static final class ElasticIntake {
        static NetworkTable intakeTable = networkInstance.getTable("intakeConstants");

        public static DoubleEntry kPOWER =
        intakeTable.getDoubleTopic("INTAKE_POWER").getEntry(Constants.IntakeConstants.kPOWER);

        public static void initialize() {
            kPOWER.set(kPOWER.get());
        }
    }
    public static final class ElasticEffector {
        static NetworkTable effectorTable = networkInstance.getTable("effectorTable");
        public static DoubleEntry kPOWER = 
        effectorTable.getDoubleTopic("POWER").getEntry(.1);
    }

    public static final class ElasticEffector {
        static NetworkTable effectorTable = networkInstance.getTable("effectorConstants");

        public static DoubleEntry kALGAE_POWER =
        effectorTable.getDoubleTopic("ALGAE_POWER").getEntry(Constants.CoralEffectorConstants.kALGAE_POWER);

        public static IntegerEntry kMILISECONDS_EXTRA_PULLBACK =
        effectorTable.getIntegerTopic("MILISECONDS_EXTRA_PULLBACK").getEntry(Constants.CoralEffectorConstants.kMILISECONDS_EXTRA_PULLBACK);
        
        public static IntegerEntry kEJECT_ALGAE_MILISECONDS =
        effectorTable.getIntegerTopic("EJECT_ALGAE_MILISECONDS").getEntry(Constants.CoralEffectorConstants.kEJECT_ALGAE_MILISECONDS);
        

        public static void initialize() {
            kALGAE_POWER.set(kALGAE_POWER.get());
            kMILISECONDS_EXTRA_PULLBACK.set(kMILISECONDS_EXTRA_PULLBACK.get());
            kEJECT_ALGAE_MILISECONDS.set(kEJECT_ALGAE_MILISECONDS.get());

        }
    }

    public static final class ElasticHumanDrive {
        static NetworkTable humanDriveTable = networkInstance.getTable("humanDriveConstants");

        public static DoubleEntry kDRIVE_POWER =
        humanDriveTable.getDoubleTopic("DRIVE_POWER").getEntry(Constants.HumanDriveConstants.kDRIVE_POWER);
        public static DoubleEntry kROTATIONAL_POWER =
        humanDriveTable.getDoubleTopic("ROTATIONAL_POWER").getEntry(Constants.HumanDriveConstants.kROTATIONAL_POWER);
        public static DoubleEntry kDRIVE_EXPONENT =
        humanDriveTable.getDoubleTopic("DRIVE_EXPONENT").getEntry(Constants.HumanDriveConstants.kDRIVE_EXPONENT);
        public static DoubleEntry kROTATIONAL_EXPONENT =
        humanDriveTable.getDoubleTopic("ROTATIONAL_EXPONENT").getEntry(Constants.HumanDriveConstants.kROTATIONAL_EXPONENT);

        public static void initialize() {
            kDRIVE_POWER.set(kDRIVE_POWER.get());
            kROTATIONAL_POWER.set(kROTATIONAL_POWER.get());
            kDRIVE_EXPONENT.set(kDRIVE_EXPONENT.get());
            kROTATIONAL_EXPONENT.set(kDRIVE_EXPONENT.get());
        }
    }

    public static void initialize() {
        ElasticElevator.initialize();
        ElasticIntake.initialize();
        ElasticSwerve.initialize();
        ElasticEffector.initialize();
        ElasticHumanDrive.initialize();
    }
}
