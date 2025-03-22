// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
        public static IntegerEntry kSLIGHTLY_DOWN_TIMEOUT =
        elevatorTable
            .getIntegerTopic("SLIGHTLY_DOWN_TIMEOUT")
            .getEntry(Constants.ElevatorConstants.kSLIGHTLY_DOWN_TIMEOUT);
        public static IntegerEntry kTIMEOUT =
        elevatorTable
            .getIntegerTopic("TIMEOUT")
            .getEntry(Constants.ElevatorConstants.kTIMEOUT);

        public static void initialize() {
            kGRAVITY_VOLTS.set(kGRAVITY_VOLTS.get());
            kMAX_VOLTS.set(kMAX_VOLTS.get());
            kMAX_VOLT_CHANGE_PER_SECOND.set(kMAX_VOLT_CHANGE_PER_SECOND.get());
            kPROPORTIONAL_VOLTS.set(kPROPORTIONAL_VOLTS.get());
            kTOLERANCE.set(kTOLERANCE.get());
            kHEIGHTS.set(kHEIGHTS.get());
            kTEST_HEIGHT.set(kTEST_HEIGHT.get());
            kSLIGHTLY_DOWN_TIMEOUT.set(kSLIGHTLY_DOWN_TIMEOUT.get());
            kTIMEOUT.set(kTIMEOUT.get());
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
        static NetworkTable effectorTable = networkInstance.getTable("effectorConstants");

        public static DoubleEntry kALGAE_POWER =
        effectorTable.getDoubleTopic("ALGAE_POWER").getEntry(Constants.CoralEffectorConstants.kALGAE_POWER);

        public static IntegerEntry kMILISECONDS_EXTRA_PULLBACK =
        effectorTable.getIntegerTopic("MILISECONDS_EXTRA_PULLBACK").getEntry(Constants.CoralEffectorConstants.kMILISECONDS_EXTRA_PULLBACK);
        
        public static IntegerEntry kEJECT_ALGAE_MILISECONDS =
        effectorTable.getIntegerTopic("EJECT_ALGAE_MILISECONDS").getEntry(Constants.CoralEffectorConstants.kEJECT_ALGAE_MILISECONDS);

        public static DoubleEntry kCORAL_POWER = 
        effectorTable.getDoubleTopic("CORAL_POWER").getEntry(Constants.CoralEffectorConstants.kCORAL_POWER);

        public static DoubleEntry kSECONDARY_OUT_POWER = 
        effectorTable.getDoubleTopic("SECONDARY_OUT_POWER").getEntry(Constants.CoralEffectorConstants.kSECONDARY_OUT_POWER);

        public static DoubleEntry kSECONDARY_IN_POWER = 
        effectorTable.getDoubleTopic("SECONDARY_IN_POWER").getEntry(Constants.CoralEffectorConstants.kSECONDARY_IN_POWER);
        

        public static void initialize() {
            kALGAE_POWER.set(kALGAE_POWER.get());
            kMILISECONDS_EXTRA_PULLBACK.set(kMILISECONDS_EXTRA_PULLBACK.get());
            kEJECT_ALGAE_MILISECONDS.set(kEJECT_ALGAE_MILISECONDS.get());
            kCORAL_POWER.set(kCORAL_POWER.get());
            kSECONDARY_IN_POWER.set(kSECONDARY_IN_POWER.get());
            kSECONDARY_OUT_POWER.set(kSECONDARY_OUT_POWER.get());
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
    public static final class ElasticClimber {
        static NetworkTable climberTable = networkInstance.getTable("climberConstants");

        public static DoubleEntry kMAX_POWER = climberTable.getDoubleTopic("MAX_POWER").getEntry(Constants.ClimberConstants.kMAX_UP_POWER);

        public static void initialize() {
            kMAX_POWER.set(kMAX_POWER.get());
        }
    }

    public static final class ElasticAlign {
        static NetworkTable alignTable = networkInstance.getTable("alignConstants");

        public static DoubleEntry kDIST_FROM_REEF = alignTable.getDoubleTopic("DIST_FROM_REEF").getEntry(Constants.AlignmentConstants.kDIST_FROM_REEF);
        public static DoubleEntry kDIST_OFFSET = alignTable.getDoubleTopic("DIST_OFFSET").getEntry(Constants.AlignmentConstants.kDIST_OFFSET);

        public static DoubleEntry kP_X = alignTable.getDoubleTopic("P_X").getEntry(Constants.AlignmentConstants.kP_X);
        public static DoubleEntry kP_Y = alignTable.getDoubleTopic("P_Y").getEntry(Constants.AlignmentConstants.kP_Y);
        public static DoubleEntry kP_ANGLE = alignTable.getDoubleTopic("P_ANGLE").getEntry(Constants.AlignmentConstants.kP_ANGLE);

        public static DoubleEntry kI_X = alignTable.getDoubleTopic("I_X").getEntry(Constants.AlignmentConstants.kI_X);
        public static DoubleEntry kI_Y = alignTable.getDoubleTopic("I_Y").getEntry(Constants.AlignmentConstants.kI_Y);
        public static DoubleEntry kI_ANGLE = alignTable.getDoubleTopic("I_ANGLE").getEntry(Constants.AlignmentConstants.kI_ANGLE);

        public static DoubleEntry kD_X = alignTable.getDoubleTopic("D_X").getEntry(Constants.AlignmentConstants.kD_X);
        public static DoubleEntry kD_Y = alignTable.getDoubleTopic("D_Y").getEntry(Constants.AlignmentConstants.kD_Y);
        public static DoubleEntry kD_ANGLE = alignTable.getDoubleTopic("D_ANGLE").getEntry(Constants.AlignmentConstants.kD_ANGLE);

        public static DoubleEntry kMAX_SPEED_X = alignTable.getDoubleTopic("MAX_SPEED_X").getEntry(Constants.AlignmentConstants.kMAX_SPEED_X);
        public static DoubleEntry kMAX_SPEED_Y = alignTable.getDoubleTopic("MAX_SPEED_Y").getEntry(Constants.AlignmentConstants.kMAX_SPEED_Y);
        public static DoubleEntry kMAX_SPEED_ANGLE = alignTable.getDoubleTopic("MAX_SPEED_ANGLE").getEntry(Constants.AlignmentConstants.kMAX_SPEED_ANGLE);
        
        public static DoubleEntry kROTATION_TIMEOUT = alignTable.getDoubleTopic("ROTATION_TIMEOUT").getEntry(Constants.AlignmentConstants.kROTATION_TIMEOUT);
        public static DoubleEntry kPOSITION_TIMEOUT = alignTable.getDoubleTopic("POSITION_TIMEOUT").getEntry(Constants.AlignmentConstants.kPOSITION_TIMEOUT);

        public static void initialize() {
            kDIST_FROM_REEF.set(kDIST_FROM_REEF.get());
            kDIST_OFFSET.set(kDIST_OFFSET.get());

            kP_X.set(kP_X.get());
            kP_Y.set(kP_Y.get());
            kP_ANGLE.set(kP_ANGLE.get());

            kI_X.set(kI_X.get());
            kI_Y.set(kI_Y.get());
            kI_ANGLE.set(kI_ANGLE.get());

            kD_X.set(kD_X.get());
            kD_Y.set(kD_Y.get());
            kD_ANGLE.set(kD_ANGLE.get());

            kMAX_SPEED_X.set(kMAX_SPEED_X.get());
            kMAX_SPEED_Y.set(kMAX_SPEED_Y.get());
            kMAX_SPEED_ANGLE.set(kMAX_SPEED_ANGLE.get());

            kROTATION_TIMEOUT.set(kROTATION_TIMEOUT.get());
            kPOSITION_TIMEOUT.set(kPOSITION_TIMEOUT.get());
        }
    }

    public static void initialize() {
        ElasticElevator.initialize();
        ElasticIntake.initialize();
        ElasticSwerve.initialize();
        ElasticEffector.initialize();
        ElasticHumanDrive.initialize();
        ElasticClimber.initialize();
        ElasticAlign.initialize();
    }
}
