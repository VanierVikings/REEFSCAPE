// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climbers m_climbers = new Climbers();
  //private final LED m_led = new LED();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driver = new CommandXboxController(0);
  final CommandXboxController operator = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser;
  PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    PDH.setSwitchableChannel(true);

    // Configure the trigger bindings
    configureBindings();
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_drivetrain,
        () -> -MathUtil.applyDeadband(driver.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getRightX(),
            OperatorConstants.RIGHT_X_DEADBAND),
        driver.getHID()::getYButtonPressed,
        driver.getHID()::getAButtonPressed,
        driver.getHID()::getXButtonPressed,
        driver.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_drivetrain.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRightX(),
        () -> driver.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = m_drivetrain.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRightX());

    Command driveFieldOrientedDirectAngleSim = m_drivetrain.simDriveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(2));

    m_drivetrain.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    autoChooser = AutoBuilder.buildAutoChooser();

    m_intake.hasNote();

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driver.a().onTrue((Commands.runOnce(m_drivetrain::zeroGyro)));
    // driver.x().onTrue(Commands.runOnce(m_drivetrain::addFakeVisionReading));
    // driver.b().whileTrue(
    // Commands.deferredProxy(() -> m_drivetrain.driveToPose(
    // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    // driver.x().whileTrue(Commands.runOnce(m_drivetrain::lock,
    // m_drivetrain).repeatedly());

    driver.leftTrigger().whileTrue(
            m_intake.shlurp(1).until(m_intake::hasNote)
                // When intake finishes, push back slightly
                .andThen(new WaitCommand(0.1))
                .andThen(m_intake.shlurp(-1).withTimeout(0.001)
                //

        ));


    driver.y().whileTrue(
        m_intake.shlurp(-1)

    );

        driver.rightTrigger().whileTrue(
            // Rev shooter
            m_shooter.prime(1)
        );

        driver.x().whileTrue(
            // Rev shooter
            m_shooter.prime(-1)
        );
        
        driver.rightBumper().whileTrue(
            // Shoot: Intake + Prime
            Commands.parallel(m_intake.shlurp(1), m_shooter.prime(1))
        );

    /*
     * operator.rightTrigger().whileTrue(new FloorIntake(m_intake, 1));
     * operator.rightBumper().whileTrue(new FloorIntake(m_intake, -1));
     * operator.leftTrigger().whileTrue(new Prime(m_shooter));
     * operator.leftBumper().whileTrue(new Shoot(m_shooter, m_intake));
     * operator.a().whileTrue(new Climb(m_climbers, 1));
     * operator.y().whileTrue(new Climb(m_climbers, -1));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    // m_drivetrain.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    m_drivetrain.setMotorBrake(brake);
  }
}
