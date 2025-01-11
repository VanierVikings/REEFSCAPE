// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

    public static final class SwerveConstants {
        public static final double MAX_SPEED  = Units.feetToMeters(14.5);
        public static final PIDConstants autoDrivePID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants autoRotationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class ElevatorConstants {
        public static final int motorOneID = 9;
        public static final int motorTwoID = 10;

        public static final double MAX_VELOCITY = 1.0; 
        public static final double MAX_ACCELERATION = 0.5; 
        public static final double ENCODER_TO_METERS = 0.01; //distance per pulse 
        
        public static final double ELEVATOR_KP = 0.1;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.0;

    }

    public static class PivotConstants {
        public static final int motorOneID = 11;
        public static final int motorTwoID = 12;    

        public static final double MAX_VELOCITY = 1.0;
        public static final double MAX_ACCELERATION = 0.5;
        public static final double ENCODER_TO_DEGREES = 0.0;

        public static final double PIVOT_KP = 0.1;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.0;

    }
}