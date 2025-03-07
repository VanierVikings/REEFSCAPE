package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkMax shooterMotor;
    private final RelativeEncoder wristEncoder;
    private final SparkClosedLoopController wristClosedLoopController;
    private double target;

    public enum SetpointEE {
        kRest,
        kSource,
        kPlaceL1,
        kPlaceL3,
        kPlaceL2,
        KAlgaeLowStart,
        KAlgaeLowEnd,
        KAlgaeHighStart,
        KAlgaeHighEnd
    }

    public EndEffector() {
        shooterMotor = new SparkMax(EndEffectorConstants.shooterMotorID, MotorType.kBrushless);
        wristMotor = new SparkMax(EndEffectorConstants.wristMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        target = EndEffectorConstants.L0_ANGLE;

        SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
        shooterMotorConfig
        .smartCurrentLimit(EndEffectorConstants.SHOOTER_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig
        .smartCurrentLimit(EndEffectorConstants.WRIST_CURRENT_LIMIT)
        .inverted(true)
        .idleMode(IdleMode.kCoast);

        wristClosedLoopController = wristMotor.getClosedLoopController();

        wristMotorConfig.encoder
                .positionConversionFactor(EndEffectorConstants.WRIST_ENCODER_TO_DEGREES);

        wristEncoder.setPosition(0);

        wristMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(EndEffectorConstants.WRIST_KP)
                .i(EndEffectorConstants.WRIST_KI)
                .d(EndEffectorConstants.WRIST_KD);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean atSetpoint(){
        return Math.abs(wristEncoder.getPosition() - target) < 1;
    }

    public Command setPosition(SetpointEE setpoint) {
        return this.runOnce(
                () -> {
                    switch (setpoint) {
                        case kRest:
                            target = EndEffectorConstants.L0_ANGLE;
                            break;
                        case kSource:
                            target = EndEffectorConstants.SOURCE_ANGLE;
                            break;
                        case kPlaceL1:
                            target = EndEffectorConstants.L1_ANGLE;
                            break;
                        case kPlaceL2:
                            target = EndEffectorConstants.L2_ANGLE;
                            break;
                        case kPlaceL3:
                            target = EndEffectorConstants.L3_ANGLE;
                            break;
                        case KAlgaeLowStart:
                            target = EndEffectorConstants.ALGAE_ANGLE_LOW_START;
                            break;
                        case KAlgaeHighEnd:
                            target = EndEffectorConstants.ALGAE_ANGLE_HIGH_END;
                            break;
                    }
                });
    }

    public Command spin(double direction) {
        return this.runEnd(() -> shooterMotor.set(direction), () -> shooterMotor.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Setpoint", target);
        wristClosedLoopController.setReference(target, ControlType.kPosition);
    }
}