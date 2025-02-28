// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
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

        public static void initializeElevator() {
            kGRAVITY_VOLTS.set(kGRAVITY_VOLTS.get());
            kMAX_VOLTS.set(kMAX_VOLTS.get());
            kMAX_VOLT_CHANGE_PER_SECOND.set(kMAX_VOLT_CHANGE_PER_SECOND.get());
            kPROPORTIONAL_VOLTS.set(kPROPORTIONAL_VOLTS.get());
            kTOLERANCE.set(kTOLERANCE.get());
            kHEIGHTS.set(kHEIGHTS.get());
        }
    }
    public static final class ElasticSwerve {
        static NetworkTable swerveTable = networkInstance.getTable("swerveTable");

        public static StructPublisher<Pose2d> kROBOT_POSITION =
            swerveTable
                .getStructTopic("ROBOT_POSITION", Pose2d.struct).publish();
        
        public static void setrobotPose(Pose2d robotPose) {
            kROBOT_POSITION.set(robotPose);
        }
    }

    public static final class ElasticIntake {
        static NetworkTable intakeTable = networkInstance.getTable("intakeConstants");

        public static DoubleEntry kPOWER =
        intakeTable.getDoubleTopic("INTAKE_POWER").getEntry(Constants.IntakeConstants.kPOWER);
        
        public static void initializeIntake() {
            kPOWER.set(kPOWER.get());
        }
    }
    public static final class ElasticEffector {
        static NetworkTable effectorTable = networkInstance.getTable("effectorTable");
        public static DoubleEntry kPOWER = 
        effectorTable.getDoubleTopic("POWER").getEntry(.1);
    }

    public static void initialize() {
        ElasticElevator.initializeElevator();
        ElasticIntake.initializeIntake();
    }
}
