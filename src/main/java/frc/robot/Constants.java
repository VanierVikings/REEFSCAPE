// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


public final class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

    public static final class SwerveConstants {
        public static final double PPTolerance = 0.25; //meters
        public static final double translationkP = 1;
        public static final double translationkI = 0.0;
        public static final double translationkD = 0.0;
        public static final double rotationalkP = 1;
        public static final double rotationalkI = 0.0;
        public static final double rotationalkD = 0.0;
        public static final double MAX_SPEED  = Units.feetToMeters(14.5);
        public static final PIDConstants autoDrivePID = new PIDConstants(4.8, 0.0, 0.0);
        public static final PIDConstants autoRotationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final double WHEEL_LOCK_TIME = 10; // seconds
        public static final double StatorCurrentLimitDrive = 100;
        public static final double SupplyCurrentLimitDrive = 60;
        public static final int SteerCurrentLimit = 40;
        public static final double branchOffset = 0.16;
        public static final double chassisOffset = 0.5;
    }

    public static final class VisionConstants {
        public static final double STDdevX = 0.7;
        public static final double STDdevY = 0.7;
        public static final double STDTheta = 10;
    }

    public static final class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class ElevatorConstants {
        public static final int motorOneID = 9; //used
        public static final int motorTwoID = 10; //used

        public static final double MAX_VELOCITY = 0.7;
        //public static final double MAX_VELOCITY = 0.75806630731;
        public static final double MAX_ACCELERATION = 4.5;
        ///public static final double MAX_ACCELERATION = 0.8717762534;
        public static final double SPOOL_DIAMETER = 0.04318;
        public static final double GEAR_RATIO = 0.0625;
        public static final double ENCODER_TO_METERS = SPOOL_DIAMETER * Math.PI * GEAR_RATIO; //distance per pulse
       
        public static final double kP = 80;
        public static final double kI = 0.0;
        public static final double kD = 0.15;
        public static final double kDt = 0.02;

        public static final double kS = 0.0;
        public static final double kV = 14.3;
        public static final double kG = 0.54;

        public static final double L0 = 0.02; //cm
        public static final double L1 = 0.15; //cm
        public static final double L2 = 0.202; //cm
        public static final double L3 = 0.34;  //cm   
        public static final double SOURCE = 0;

        public static final int ELEVATOR_CURRENT_LIMIT = 40; //amps

    }

    public static final class PivotConstants {
        public static final int motorOneID = 11; //used
        public static final int motorTwoID = 12; //used

        public static final double MAX_VELOCITY = 0.4;
        public static final double MAX_ACCELERATION = 4;

        public static final double kP = 200;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kDt = 0.02;

        public static final double kA = 0;
        public static final double kS = 0;
        public static final double kV = 31;
        public static final double kG = 0;

        public static final double L0_ANGLE = 10; //degrees
        public static final double L1_ANGLE = 90; //degrees
        public static final double L2_ANGLE = 68; //degrees
        public static final double L3_ANGLE = 78.5; //degrees
        public static final double HANG_ANGLE = 90;
        public static final double SOURCE_ANGLE = 0; //degrees
        
        public static final int PIVOT_CURRENT_LIMIT = 40; //amps
    }

    public static final class EndEffectorConstants {
        public static final int shooterMotorID = 13; //used
        public static final int wristMotorID = 14; //used

        public static final double WRIST_KI = 0;
        public static final double WRIST_KP = 0.04;
        public static final double WRIST_KD = 0;

        public static final double WRIST_ENCODER_TO_DEGREES = 5.72727272; //degrees

        public static final double L0_ANGLE = 10;
        public static final double SOURCE_ANGLE = 0;
        public static final double L1_ANGLE = 90;
        public static final double LGEN_ANGLE = 90;
        public static final double L2_ANGLE = 79.2;


        public static final int WRIST_CURRENT_LIMIT = 40; //amps
        public static final int SHOOTER_CURRENT_LIMIT = 40; //amps
    }

}