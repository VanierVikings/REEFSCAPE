package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
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
        kPlaceGen
    }

    public EndEffector() {
        shooterMotor = new SparkMax(EndEffectorConstants.shooterMotorID, MotorType.kBrushless);
        wristMotor = new SparkMax(EndEffectorConstants.wristMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        target = EndEffectorConstants.L0_ANGLE;

        SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
        shooterMotorConfig
        .smartCurrentLimit(EndEffectorConstants.SHOOTER_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12);

        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig
        .smartCurrentLimit(EndEffectorConstants.WRIST_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12);

        wristClosedLoopController = wristMotor.getClosedLoopController();

        wristMotorConfig.encoder
                .positionConversionFactor(EndEffectorConstants.WRIST_ENCODER_TO_DEGREES);

        wristMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(EndEffectorConstants.WRIST_KP)
                .i(EndEffectorConstants.WRIST_KI)
                .d(EndEffectorConstants.WRIST_KD);

        this.setDefaultCommand(this.run(() -> wristClosedLoopController.setReference(wristEncoder.getPosition(), ControlType.kPosition)));
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
                        case kPlaceGen:
                            target = EndEffectorConstants.LGEN_ANGLE;
                            break;
                    }
                });
    }

    public Command spin(int direction) {
        return this.run(() -> shooterMotor.set(direction));
    }
}