// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Ball_arm;
import frc.robot.subsystems.Coral_arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake_Algea;
import frc.robot.subsystems.Intake_Coral;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController audreyXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final Ball_arm Ball_arm = new Ball_arm();
  private final Coral_arm Coral_arm = new Coral_arm();
  private final Intake_Algea Intake_Algea = new Intake_Algea();
  private final Intake_Coral intake_Coral = new Intake_Coral();
  private final Elevator Elevator = new Elevator();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    if (Robot.isSimulation()) {
      driverXbox
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    driverXbox
        .a()
        .onTrue(
            Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    driverXbox
        .rightBumper()
        .onTrue(Commands.runOnce(() -> Elevator.setTargetSpeed(0.7)))
        .onFalse(Commands.runOnce(() -> Elevator.setTargetSpeed(0.0)));

    driverXbox
        .leftBumper()
        .onTrue(Commands.runOnce(() -> Elevator.setTargetSpeed(-0.7)))
        .onFalse(Commands.runOnce(() -> Elevator.setTargetSpeed(0.0)));

    // Send the coral slowly
    driverXbox
        .b()
        .onTrue(Commands.runOnce(() -> intake_Coral.setTargetSpeed(-0.3)))
        .onFalse(Commands.runOnce(() -> intake_Coral.setTargetSpeed(0.0)));

    audreyXbox
        .rightBumper()
        .onTrue(Commands.runOnce(() -> Ball_arm.setTargetSpeed(-0.8)))
        .onFalse(Commands.runOnce(() -> Ball_arm.setTargetSpeed(0.0)));
    audreyXbox
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> Ball_arm.setTargetSpeed(0.8)))
        .onFalse(Commands.runOnce(() -> Ball_arm.setTargetSpeed(0.0)));
    audreyXbox
        .leftBumper()
        .onTrue(Commands.runOnce(() -> Intake_Algea.setTargetSpeed(0.9)))
        .onFalse(Commands.runOnce(() -> Intake_Algea.setTargetSpeed(0.0)));
    audreyXbox
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> Intake_Algea.setTargetSpeed(-0.9)))
        .onFalse(Commands.runOnce(() -> Intake_Algea.setTargetSpeed(0.0)));
    audreyXbox
        .x()
        .onTrue(Commands.runOnce(() -> intake_Coral.setTargetSpeed(-0.5)))
        .onFalse(Commands.runOnce(() -> intake_Coral.setTargetSpeed(0.0)));

    audreyXbox
        .y()
        .onTrue(Commands.runOnce(() -> intake_Coral.setTargetSpeed(0.8)))
        .onFalse(Commands.runOnce(() -> intake_Coral.setTargetSpeed(0.0)));
    audreyXbox
        .a()
        .whileTrue(Commands.run(() -> Coral_arm.setTargetPosition(-0.07)))
        .onFalse(Commands.runOnce(() -> Coral_arm.stop()));
    audreyXbox
        .start()
        .whileTrue(Commands.run(() -> Coral_arm.setTargetPosition(0.0)))
        .onFalse(Commands.runOnce(() -> Coral_arm.stop()));
    audreyXbox
        .povUp()
        .whileTrue(Commands.run(() -> Coral_arm.setTargetPosition(0.54)))
        .onFalse(Commands.runOnce(() -> Coral_arm.stop()));
    audreyXbox
        .b()
        .whileTrue(Commands.run(() -> Coral_arm.setTargetPosition(0.4)))
        .onFalse(Commands.runOnce(() -> Coral_arm.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
