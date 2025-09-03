// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.PS4Driver.DEADBAND_LARGE;
import static frc.robot.settings.Constants.PS4Driver.DEADBAND_NORMAL;
import static frc.robot.settings.Constants.PS4Driver.NO_INPUT;
import static frc.robot.settings.Constants.PS4Driver.X_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Y_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Z_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Z_ROTATE;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Drive;

import frc.robot.settings.Constants;

import frc.robot.subsystems.DrivetrainSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private DrivetrainSubsystem drivetrain;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private final PS4Controller driveController;
  private final PS4Controller opController;


  public static HashMap<String, Command> eventMap;

  public RobotContainer() {
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    driveController = new PS4Controller(0);
    opController = new PS4Controller(1);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();

    drivetrain = new DrivetrainSubsystem();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> Robot X movement, backward/forward
    // Left stick X axis -> Robot Y movement, right/left
    // Right stick Z axis -> rotation, clockwise, counterclockwise
    // Need to invert the joystick axis
    defaultDriveCommand = new Drive(
        drivetrain,
        () -> driveController.getL1Button(),
        () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
        () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
        () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
    drivetrain.setDefaultCommand(defaultDriveCommand);
    SmartDashboard.putBoolean("use limelight", false);
    SmartDashboard.putBoolean("trust limelight", false);
    autoInit();
    configureBindings();
  }

  private void autoInit() {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Takes both axis of a joystick, returns an angle from -180 to 180 degrees, or
   * {@link Constants.PS4Driver.NO_INPUT} (double = 404.0) if the joystick is at
   * rest position
   */
  private double getJoystickDegrees(int horizontalAxis, int verticalAxis) {
    double xAxis = MathUtil.applyDeadband(-driveController.getRawAxis(horizontalAxis), DEADBAND_LARGE);
    double yAxis = MathUtil.applyDeadband(-driveController.getRawAxis(verticalAxis), DEADBAND_LARGE);
    if (xAxis + yAxis != 0) {
      return Math.toDegrees(Math.atan2(xAxis, yAxis));
    }
    return NO_INPUT;
  }

  /** Takes both axis of a joystick, returns a double from 0-1 */
  private double getJoystickMagnitude(int horizontalAxis, int verticalAxis) {
    double xAxis = MathUtil.applyDeadband(-driveController.getRawAxis(horizontalAxis), DEADBAND_NORMAL);
    double yAxis = MathUtil.applyDeadband(-driveController.getRawAxis(verticalAxis), DEADBAND_NORMAL);
    return Math.min(1.0, (Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2)))); // make sure the number is not greater than 1
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4DriverController
   * PS4Driver} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(driveController::getPSButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  public void robotInit() {
      drivetrain.zeroGyroscope();
  }

  public void teleopInit() {
  }

  public void teleopPeriodic() {
  }
}
