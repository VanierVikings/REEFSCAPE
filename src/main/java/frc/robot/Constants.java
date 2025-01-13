// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    public static final class IntakeConstants {

    SparkMax intakePivot = new SparkMax(12, MotorType.kBrushless);
    SparkMax intakeRollers = new SparkMax(13, MotorType.kBrushless);
    SparkMax intakeIndexer = new SparkMax(14, MotorType.kBrushless);
    DutyCycleEncoder angleMeasure = new DutyCycleEncoder(1);
    DigitalInput beamBreak = new DigitalInput(9);

    }
}

