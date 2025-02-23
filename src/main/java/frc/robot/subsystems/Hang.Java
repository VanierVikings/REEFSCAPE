package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.hangConstants;

public class Hang extends SubsystemBase{

private SparkMax grabber = new SparkMax(16, MotorType.kBrushless); 
 

public Hang(){
    SparkMax grabber = new SparkMax(hangConstants.CANID, MotorType.kBrushless);
}

//public boolean isGrabbed(){
    
//}

public Command hang(int speed){
    return run(() -> grabber.set(speed)).withTimeout(hangConstants.SECONDS).andThen(() -> grabber.set(0));
}




}   