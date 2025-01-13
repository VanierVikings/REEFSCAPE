import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class CoralIntake {
    SparkMax intakePivot;
    SparkMax intakeRollers;
    SparkMax intakeIndexer;
    DutyCycleEncoder angleMeasure;
    DigitalInput beamBreak;
    SparkClosedLoopController pivotController;
    SparkMaxConfig config;
    double maxVel;
    double maxAccel;
    double allowedErr;

    public CoralIntake() {
        intakePivot = new SparkMax(12, MotorType.kBrushless);
        intakeRollers = new SparkMax(13, MotorType.kBrushless);
        intakeIndexer = new SparkMax(14, MotorType.kBrushless);
        angleMeasure = new DutyCycleEncoder(1);
        beamBreak = new DigitalInput(9);
        pivotController = new intakePivot.getClosedLoopController();
        angleMeasure = new DutyCycleEncoder(1);
        config = new SparkMaxConfig();
        config.closedloop.maxMotion.maxVelocity(maxVel).maxAcceleration(maxAccel).allowedClosedLoopError(allowedErr);

    }

public Boolean coralGrabbed(){
    return !beamBreak.get();
   
}


public void pivoter(double targetPosition){
    // pivates the intake down to colllect coral
    pivotController.setReference(targetPosition-angleMeasure.get.distance(), SparkMax.ControlType.kMAXMotionPositionControl);

} 

public void runIntake(){
    Commands.parallel(pivoter(1.0), intakeRollers.set(-1)).until(coralGrabbed()).andThen(pivoter(1)).andThen(intakeIndexer.set(1));
}

}


