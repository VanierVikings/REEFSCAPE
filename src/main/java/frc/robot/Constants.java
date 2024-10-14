// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.module.FindException;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class IntakeConstants {
    public static final int topSpeed = -1;
    public static final int bottomSpeed = 1;
  }

  public static final class ClimbConstants {
    public static final double leftSpeed = 0.2;
    public static final double rightSpeed = 0.5;
  }

  public static final class ShooterConstants {
    public static final double kP = 1.0;
    public static final double kI = 0.01;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0.26;
    public static final double kA = 0.14;
    public static final double maxRPM = 5800;
    public static final int currentLimit = 40;
    public static final int tolerance = 50;

  }

  public final class LEDConstants {
    public static final int PWM_PORT = 1;
    public static final int NUM_LED = 30;
    public static final int RAINBOW_START_INDEX = 0;
    public static final int RAINBOW_END_INDEX = 100;
  }

  public static final class DeviceIDs {
    public static final int intakeBottom = 20;
    public static final int intakeTop = 21;
    public static final int shooterLeft = 22;
    public static final int shooterRight = 23;
    public static final int leftClimbSwitch = 8;
    public static final int rightClimbSwitch = 9;
    public static final int leftClimb = 24;
    public static final int rightClimb = 25;

  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 15; // seconds
    public static final int StatorCurrentLimitDrive = 100;
    public static final int SupplyCurrentLimitDrive = 60;
    public static final int SteerCurrentLimit = 35;
    public static final int outputGear = 16;
    public static final int pinionGear = 14;
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.5;
    public static final double TURN_CONSTANT = 6;
  }
}
