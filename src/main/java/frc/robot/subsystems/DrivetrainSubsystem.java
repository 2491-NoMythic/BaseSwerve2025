// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kP;
import static frc.robot.settings.Constants.DriveConstants.BL_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BL_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.BL_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.settings.Constants.DriveConstants.DRIVE_ODOMETRY_ORIGIN;
import static frc.robot.settings.Constants.DriveConstants.DRIVE_TO_POSE_X_CONTROLLER;
import static frc.robot.settings.Constants.DriveConstants.DRIVE_TO_POSE_Y_CONTROLLER;
import static frc.robot.settings.Constants.DriveConstants.FL_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FL_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.FL_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.settings.Constants.DriveConstants.ROBOT_ANGLE_TOLERANCE;


import java.util.Arrays;
import java.util.Collections;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// import java.util.logging.Logger;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants;


public class DrivetrainSubsystem extends SubsystemBase {
  //These are our swerve drive kinematics and Pigeon (gyroscope)
  public SwerveDriveKinematics kinematics = DriveConstants.kinematics;

  private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, CANIVORE_DRIVETRAIN);

  /**
   * These are our modules. We initialize them in the constructor. 0 = Front Left 1 = Front Right 2 = Back Left 3 = Back Right
   */
  private final SwerveModule[] modules;
  /**
	 * These are the angles the wheels are at. This is mostly used to stop the robot without changing the angles.
	 */
  private final Rotation2d[] lastAngles;
  /**
	 * This is a number that keeps track of how many times the steering motor has rotated.
	 */
  private int accumulativeLoops;
	/**
	 * This is the odometer.
	 */
  private final SwerveDrivePoseEstimator odometer;
  private final Field2d m_field = new Field2d();

 
  PIDController speedController;
  PIDController rotationSpeedController;

  public DrivetrainSubsystem() {
    rotationSpeedController = new PIDController(AUTO_AIM_ROBOT_kP, AUTO_AIM_ROBOT_kI, AUTO_AIM_ROBOT_kD);
    rotationSpeedController.setTolerance(ROBOT_ANGLE_TOLERANCE);
    rotationSpeedController.enableContinuousInput(-180, 180);
    Preferences.initDouble("FL offset", 0);
    Preferences.initDouble("FR offset", 0);
    Preferences.initDouble("BL offset", 0);
    Preferences.initDouble("BR offset", 0);
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> m_field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Vision/force use limelight", false);

    // Creates and configures each of the four swerve modules used in the drivetrain, along with their motor loggers.
    modules = new SwerveModule[4];
    lastAngles =
        new Rotation2d[] {
          new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()
        }; // manually make empty angles to avoid null errors.

    modules[0] =
        new SwerveModule(
            "FL",
            FL_DRIVE_MOTOR_ID,
            FL_STEER_MOTOR_ID,
            FL_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("FL offset", 0)),
            CANIVORE_DRIVETRAIN);
    modules[1] =
        new SwerveModule(
            "FR",
            FR_DRIVE_MOTOR_ID,
            FR_STEER_MOTOR_ID,
            FR_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("FR offset", 0)),
            CANIVORE_DRIVETRAIN);
    modules[2] =
        new SwerveModule(
            "BL",
            BL_DRIVE_MOTOR_ID,
            BL_STEER_MOTOR_ID,
            BL_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("BL offset", 0)),
            CANIVORE_DRIVETRAIN);
    modules[3] =
        new SwerveModule(
            "BR",
            BR_DRIVE_MOTOR_ID,
            BR_STEER_MOTOR_ID,
            BR_STEER_ENCODER_ID,
            Rotation2d.fromRotations(Preferences.getDouble("BR offset", 0)),
            CANIVORE_DRIVETRAIN);

    // configures the odometer
    odometer =
        new SwerveDrivePoseEstimator(
            kinematics, getGyroscopeRotation(), getModulePositions(), DRIVE_ODOMETRY_ORIGIN);
    odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 99999999));
  }
  // This is the main 'get' section
	/**
	 * Gets the robot pose.
	 * @return
	 */
  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }
  /**
	 * Returns the gyroscope rotation.
	 * @return
	 */
	public Rotation2d getGyroscopeRotation() {
		return pigeon.getRotation2d();
	}
  /**
   * gets the angle of odometer reading, but adds 180 degrees if we are on red alliance. this is useful for whne using ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds, getAllianceSpecificRotation())
   * @return the angle of the robot, if 0 degrees is away from your alliance wall
   */
  private Rotation2d getAllianceSpecificRotation() {
    double angle = DriverStation.getAlliance().get() == Alliance.Blue ? odometer.getEstimatedPosition().getRotation().getDegrees() : odometer.getEstimatedPosition().getRotation().getDegrees() + 180;
    return Rotation2d.fromDegrees(angle);
  }
  /**
   * returns the pitch of the pigeon as a double
   * @return the pitch, returned as a double
   */
  public double getPigeonPitch(){
    double pitch = pigeon.getPitch().getValueAsDouble();
    return pitch;
  }

  public double getPigeonRoll(){
    double roll = pigeon.getRoll().getValueAsDouble();
    return roll;
  }
  
  /**
   * @return a rotation2D of the angle according to the odometer
   */
  public Rotation2d getOdometryRotation() {
    return odometer.getEstimatedPosition().getRotation();
  }
  /**
	 * Returns the angle as degrees instead of rotations
	 * @return the angle in degreeds instead of rotations
	 */
	public double headingAsDegrees(){
		return getOdometryRotation().getDegrees();
	}
  /** 
   * Returns the heading of the robot, but only out of 360, not accumulative 
   * */
  public double getHeadingLooped() {
    accumulativeLoops =
        (int)
            (headingAsDegrees()
                / 180); // finding the amount of times that 360 goes into the heading, as an int
    return headingAsDegrees() - 180 * (accumulativeLoops);
  }
  	/**
	 * Returns what directions the swerve modules are pointed in
	 * @return the positions of the swerve modules
	 */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
    return positions;
  }
	/**
	 * Returns the swerve module states (a mxiture of speed and angle)
	 * @return the speeds and angles of the swerve modules
	 */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
    return states;
  }
  /**
   * Gets the speed of the robot
   * @return the module states in terms of speed
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  //This is the odometry section. It has odometry-related functions.
  /**
	 * Resets the odometry of the robot.
	 * @param pose
	 */
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
  } 
  /**
   * Sets the gyro to the specified position.
   */
  public void setGyroscope(double angleDeg) {
    resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
  }
  /**
   * Sets the gyroscope angle to zero.  
   */
  public void zeroGyroscope() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      setGyroscope(180);
    } else {
      setGyroscope(0);
    }
  }
  //This is the set section that sends commands to the modules
  /**
   * Calls the findOffset fucntion for each module.
   */
  public void setEncoderOffsets() {
    Preferences.setDouble("FL offset", modules[0].findOffset());
    Preferences.setDouble("FR offset", modules[1].findOffset());
    Preferences.setDouble("BL offset", modules[2].findOffset());
    Preferences.setDouble("BR offset", modules[3].findOffset());
  }
  /**
   * Sets a given module to a given module state. 
   * @param i the ID of the module
   * @param desiredState the speed and angle as a SwerveModuleState
   */
  private void setModule(int i, SwerveModuleState desiredState) {
    modules[i].setDesiredState(desiredState);
    lastAngles[i] = desiredState.angle;
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
    for (int i = 0; i < 4; i++) {
      setModule(i, desiredStates[i]);
    }
  }
  /** Sets the modules speed and rotation to zero. */
  //TODO: Make a version that works with States. 
  public void pointWheelsForward() {
    for (int i = 0; i < 4; i++) {
      setModule(i, new SwerveModuleState(0, new Rotation2d()));
    }
  }
  /**
   * Points all of the wheels towards the center of the robot, making it harder to push. 
   */
  public void pointWheelsInward() {
    setModule(0, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)));
    setModule(1, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    setModule(2, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    setModule(3, new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
   /**
   * Stops the robot. 
   */
  public void stop() {
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(new SwerveModuleState(0, lastAngles[i]));
    }
  }
  /**
   * The function that actually lets us drive the robot.
   * @param chassisSpeeds the desired speed and direction
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    if (Preferences.getBoolean("AntiTipActive", false)) {
      if (pigeon.getRoll().getValueAsDouble() > 3) {
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
            + (pigeon.getRoll().getValueAsDouble()/10);
      } else if (pigeon.getRoll().getValueAsDouble() < -3) {
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
            + (-pigeon.getRoll().getValueAsDouble()/10);
      }
      if (pigeon.getPitch().getValueAsDouble() > 3) {
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
            + (pigeon.getPitch().getValueAsDouble()/10);
      } else if (pigeon.getPitch().getValueAsDouble() < -3) {
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
            + (-pigeon.getPitch().getValueAsDouble()/10);
      }
    }
      if (DriverStation.isTest()){
       chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond / 4; 
       chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond / 4; 
       chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond / 4;
      }
    SwerveModuleState[] desiredStates =
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));
    double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
    if (maxSpeed <= DriveConstants.DRIVE_DEADBAND_MPS) {
      for (int i = 0; i < 4; i++) {
        stop();
      }
    } else {
      setModuleStates(desiredStates);
    }
}
}