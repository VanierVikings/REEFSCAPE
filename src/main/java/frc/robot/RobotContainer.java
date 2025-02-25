// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import swervelib.SwerveController;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Setpoint;
import frc.robot.subsystems.EndEffector.SetpointEE;
import frc.robot.subsystems.EndEffector;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final static Elevator m_elevator = new Elevator();
  private final static EndEffector m_endEffector = new EndEffector();
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivetrain = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveController controller = drivetrain.getSwerveController();
  SwerveInputStream reefPoint = new SwerveInputStream(drivetrain.getSwerveDrive(), () -> driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX(),
      () -> controller.headingCalculate(drivetrain.getHeading().getRadians(), drivetrain.getReefTag().getRotation().getRadians()));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityRP = drivetrain.driveFieldOriented(reefPoint);
    // Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    // Command driveRobotOrientedAngularVelocity = drivetrain.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivetrain.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngle);

    if (RobotBase.isSimulation()) {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      driverXbox.start().onTrue(Commands.runOnce(() -> drivetrain.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    } else {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    m_elevator.setDefaultCommand(m_elevator.moveToSetpoint());
   
    // driverXbox.b().onTrue(m_elevator.setElevator(Setpoint.rest)
    // .andThen(m_elevator.setPivot(Setpoint.rest)));

    // driverXbox.a().onTrue(m_elevator.setPivot(Setpoint.kLevel1).andThen(m_elevator.setElevator(Setpoint.kLevel1)));

    // // X Button -> Elevator/Arm to level 2 position
    // driverXbox.x().onTrue(m_elevator.setPivot(Setpoint.kLevel2).andThen(m_elevator.setElevator(Setpoint.kLevel2))); 

    //driverXbox.x().whileTrue(driveFieldOrientedAnglularVelocityRP);
    driverXbox.x().onTrue(m_elevator.setElevator(Setpoint.kLevel1));

    // SwerveController controller = drivetrain.getSwerveController();
    // driverXbox.x()
    //     .whileTrue(drivetrain.driveWithSetpointGeneratorFieldRelative(() -> new ChassisSpeeds(
    //         drivetrain.getFieldVelocity().vxMetersPerSecond, drivetrain.getFieldVelocity().vyMetersPerSecond,
    //         controller.headingCalculate(drivetrain.getHeading().getRadians(), drivetrain.getReefTag().getRadians()))));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivetrain.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivetrain.setMotorBrake(brake);
  }  
 
 
}