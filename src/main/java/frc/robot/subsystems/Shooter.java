package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    CANSparkMax flyWheelLeft = new CANSparkMax(DeviceIDs.shooterLeft, MotorType.kBrushless);
    CANSparkMax flyWheelRight = new CANSparkMax(DeviceIDs.shooterRight, MotorType.kBrushless);

    public Shooter() {
    
    }

    public void runMotors(int direction) {
        // Set fly wheel speeds
        flyWheelLeft.set(-1 * direction);
        flyWheelRight.set(1 * direction);
    }

    public void stopMotors() {
        // Set fly wheel speeds
        flyWheelLeft.set(0);
        flyWheelRight.set(0);
    }

    public Command prime(int direction) {
        //PRIME LIKE LUNCHLY
        return this.startEnd(() -> this.runMotors(direction), () -> this.stopMotors());
    } 

}