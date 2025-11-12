package frc.robot.LogInputs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * this is the class where we record all the inputs to be recorded for replay simulation. These are inputs that we use with logic and math in our 
 * commands and subsystems. 
 * <p>
 * This class should always be used when referencing the recorded inputs, rather than using the sensors themselves. When using this class, 
 * there are 3 places that it needs to be incorporated into code. 
 * <p>
 * 1) create an instance of this class (using the tag "AutoLogged" after the class name) In the periodic method of the associated subsystem,
 * update each input by reading the sensors. Then, write the line {@code Logger.ProcessInputs("SubsystemName/", instance of this class)}
 * <p>
 * 2) anytime these inputs are used to produce an output - via logic, math, or raw value - reference them via {@code thisClass.inputName} rather than using
 * the sensors themselves. This will allow outputs to be changed and monitored during replay simulation
 * <p>
 * 3) when wanting to see how these outputs change during replay simulation, make sure the monitored output is recorded with {@code Logger.RecordOutput("entryName", value)}
 */
@AutoLog
public class DrivetrainInputs {
    public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    public Rotation2d gyroScopeRotation = new Rotation2d();
    public double pitch;
    public double roll;
    public double gyroTimeStamp;
    public double angularVelocity;
}
