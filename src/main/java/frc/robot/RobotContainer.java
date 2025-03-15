// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.branchSide;

import java.io.File;
import java.lang.System.Logger.Level;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveController;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Setpoint;
import frc.robot.subsystems.EndEffector.SetpointEE;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Climb;


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
  final CommandXboxController operator = new CommandXboxController(1);
  final CommandXboxController driver = new CommandXboxController(0);
  private final static Elevator m_elevator = new Elevator();
  private final static EndEffector m_endEffector = new EndEffector();
  private final static Climb m_hang = new Climb();
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivetrain = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
     "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX()) // Axis which give the desired translational angle and speed.
  .withControllerRotationAxis(() -> -driver.getRightX()) // Axis which give the desired angular velocity.
  .deadband(0.005)                  // Controller deadband
  .scaleTranslation(0.8)           // Scaled controller translation axis
  .allianceRelativeControl(true);  // Alliance relative controls.

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> Math.cos(Math.PI/3), () -> Math.sin(Math.PI/3)).headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream reefPoint = new SwerveInputStream(drivetrain.getSwerveDrive(), () -> -driver.getLeftY(),
      () -> -driver.getLeftX(),
      () -> drivetrain.controller.headingCalculate(drivetrain.getHeading().getRadians(), drivetrain.getNearestReefPose().getRotation().getRadians()))
      .deadband(0.005)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("scoreL1", m_elevator.setPivot(Setpoint.kLevel1).andThen(Commands.waitSeconds(0.25)).andThen(m_elevator.setElevator(Setpoint.kLevel1)).andThen(Commands.waitSeconds(0.25)).andThen(m_endEffector.setPosition(SetpointEE.kPlaceL1)).andThen(m_endEffector.spin(0.4).withTimeout(0.5))); 
    NamedCommands.registerCommand("scoreL2", m_elevator.setPivot(Setpoint.kLevel2).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kLevel2)).andThen(Commands.waitSeconds(0.5)).andThen(m_endEffector.setPosition(SetpointEE.kPlaceL2)).andThen(m_endEffector.spin(0.7).withTimeout(1)));
    NamedCommands.registerCommand("scoreL3", m_elevator.setPivot(Setpoint.kLevel3).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kLevel3)).andThen(Commands.waitSeconds(0.5)).andThen(m_endEffector.setPosition(SetpointEE.kPlaceL3)).andThen(m_endEffector.spin(0.7).withTimeout(1)));
    NamedCommands.registerCommand("scoreSource", m_elevator.setPivot(Setpoint.kSource).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kSource)).andThen(Commands.waitSeconds(0.5)).andThen(m_endEffector.setPosition(SetpointEE.kSource)).andThen(m_endEffector.spin(0.7).withTimeout(2.5)));  
    NamedCommands.registerCommand("rest", m_endEffector.setPosition(SetpointEE.kRest).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kRest)).andThen(Commands.waitSeconds(0.35).andThen(m_elevator.setPivot(Setpoint.kRest))));
    autoChooser = AutoBuilder.buildAutoChooser();  
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    DriverStation.silenceJoystickConnectionWarning(true);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * 
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityRP = drivetrain.driveFieldOriented(reefPoint);
    Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngularVelocity = drivetrain.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivetrain.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);


    if (RobotBase.isSimulation()) {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      drivetrain.resetOdometry(new Pose2d(3,3,new Rotation2d()));
      drivetrain.visionEnabled = false;
    } else {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    driver.rightBumper().toggleOnTrue(driveFieldOrientedAnglularVelocityRP);

    driver.a().onTrue(drivetrain.runOnce(drivetrain::zeroGyroWithAlliance));

    operator.leftTrigger().onTrue(m_endEffector.setPosition(SetpointEE.kRest).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kRest)).andThen(Commands.waitSeconds(0.35).andThen(m_elevator.setPivot(Setpoint.kRest))));
    operator.leftBumper().onTrue(m_elevator.setPivot(Setpoint.kLevel1).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kLevel1)).andThen(Commands.waitSeconds(0.25)).andThen(m_endEffector.setPosition(SetpointEE.kPlaceL1)));
    operator.rightBumper().onTrue(m_elevator.setPivot(Setpoint.kLevel2).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kLevel2)).andThen(Commands.waitSeconds(0.25)).andThen(m_endEffector.setPosition(SetpointEE.kPlaceL2)));
    operator.rightTrigger().onTrue(m_elevator.setPivot(Setpoint.kLevel3).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kLevel3)).andThen(Commands.waitSeconds(0.25)).andThen(m_endEffector.setPosition(SetpointEE.kPlaceL3)));
    operator.b().onTrue(m_elevator.setPivot(Setpoint.kSource).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.kSource)).andThen(Commands.waitSeconds(0.5)).andThen(m_endEffector.setPosition(SetpointEE.kSource)));
    //low reef algae removal
    operator.a().onTrue(m_elevator.setPivot(Setpoint.KAlgaeLowStart).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.KAlgaeLowStart)).andThen(Commands.waitSeconds(0.2)).andThen(m_endEffector.setPosition(SetpointEE.kRest).andThen(Commands.waitSeconds(0)).andThen(m_elevator.setPivot(Setpoint.KAlgaeLowEnd).andThen(Commands.waitSeconds(0.1)).andThen(m_elevator.setElevator(Setpoint.KAlgaeLowEnd)))));
    //high reef algae removal
    operator.y().onTrue(m_elevator.setPivot(Setpoint.KAlgaeHighStart).andThen(Commands.waitSeconds(0.2)).andThen(m_elevator.setElevator(Setpoint.KAlgaeHighStart)).andThen(Commands.waitSeconds(0.2)).andThen(m_endEffector.setPosition(SetpointEE.kRest).andThen(Commands.waitSeconds(0)).andThen(m_elevator.setPivot(Setpoint.KAlgaeHighEnd).andThen(Commands.waitSeconds(0.1)).andThen(m_elevator.setElevator(Setpoint.KAlgaeHighEnd)))));
    operator.povUp().whileTrue(m_endEffector.spin(0.65));
    operator.povLeft().whileTrue(m_endEffector.spin(0.3));
    operator.povDown().whileTrue(m_endEffector.spin(-1));

    operator.povRight().whileTrue(m_elevator.run(() -> m_elevator.elevatorController.setGoal(m_elevator.elevatorController.getGoal().position - 0.001)));

    driver.y().whileTrue(drivetrain.defer(() -> drivetrain.autoAlign(drivetrain.getBranchPose(branchSide.leftBranch))));
    driver.b().whileTrue(drivetrain.defer(() -> drivetrain.autoAlign(drivetrain.getBranchPose(branchSide.rightBranch))));

    // driver.x().onTrue(m_elevator.setPivot(Setpoint.kHang));
    // driver.y().onTrue(m_hang.setpoint());
    // driver.b().onTrue((m_elevator.setPivot(Setpoint.kClimb)).andThen(m_elevator.setElevator(Setpoint.kLevel2)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autus
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //   return drivetrain.driveFieldOriented(SwerveInputStream.of(drivetrain.getSwerveDrive(),
  //   () -> 1,
  //   () -> 0) // Axis which give the desired translational angle and speed.
  // .withControllerRotationAxis(() -> 0) // Axis which give the desired angular velocity.
  // .deadband(0.05)                  // Controller deadband
  // .scaleTranslation(0.8)           // Scaled controller translation axis
  // .allianceRelativeControl(true));
  return autoChooser.getSelected();
    }


  public void setMotorBrake(boolean brake) {
    drivetrain.setMotorBrake(brake);
  }  
 
 
}