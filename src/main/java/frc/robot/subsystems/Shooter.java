package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkBase shooterPrimeRight = new CANSparkMax(DeviceIDs.shooterRight,
            MotorType.kBrushless);
    private final CANSparkBase shooterPrimeLeft = new CANSparkMax(DeviceIDs.shooterLeft,
            MotorType.kBrushless);

    private RelativeEncoder encoderLeft = shooterPrimeLeft.getEncoder();
    private RelativeEncoder encoderRight = shooterPrimeRight.getEncoder();

    private final PIDController m_leftPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);
    private final PIDController m_rightPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ShooterConstants.kS,
      ShooterConstants.kV, ShooterConstants.kA);

    public Shooter() {
        shooterPrimeLeft.restoreFactoryDefaults();  
        shooterPrimeRight.restoreFactoryDefaults();
        shooterPrimeLeft.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterPrimeRight.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterPrimeLeft.setIdleMode(IdleMode.kBrake);
        shooterPrimeRight.setIdleMode(IdleMode.kBrake);
        shooterPrimeLeft.setInverted(true);
        shooterPrimeRight.setInverted(true);

        m_leftPIDController.setTolerance(ShooterConstants.tolerance);
        m_rightPIDController.setTolerance(ShooterConstants.tolerance);
    }



    public void set(double setpoint) {
        m_leftPIDController.reset();
        m_rightPIDController.reset();

        double feedforward = m_feedforward.calculate(setpoint);
        m_leftPIDController.setSetpoint(setpoint);
        double leftOutput = m_leftPIDController.calculate(encoderLeft.getVelocity(), setpoint);
        double rightOutput = m_rightPIDController.calculate(encoderRight.getVelocity(), setpoint);
        
        shooterPrimeLeft.setVoltage(-(leftOutput + feedforward)*shooterPrimeLeft.getBusVoltage());
        shooterPrimeRight.setVoltage((rightOutput + feedforward)*shooterPrimeRight.getBusVoltage());
    }   

    public boolean atSetpoint() {
        return m_rightPIDController.atSetpoint();
    }

    public void extend() {
    }

    public void stop() {
        shooterPrimeRight.stopMotor();
        shooterPrimeLeft.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LEFT Shooter Encoder", encoderLeft.getVelocity());
        SmartDashboard.putNumber("RIGHT Shooter Encoder", encoderRight.getVelocity());
        SmartDashboard.putBoolean("At Max RPM?", atSetpoint());
    }
}

