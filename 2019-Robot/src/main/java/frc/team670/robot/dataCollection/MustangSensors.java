package frc.team670.robot.dataCollection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.NavX;
import frc.team670.robot.dataCollection.sensors.NavX.NavX_Pitch_PIDSource;
import frc.team670.robot.dataCollection.sensors.NavX.ZeroableNavX_Yaw_PIDSource;
import frc.team670.robot.utils.math.Rotation;

/**
 * Instantiates sensor representation objects and contains methods for accessing the sensor data.
 * @author shaylandias
 */
public class MustangSensors extends Subsystem {

  // NavX
  private NavX navXMicro = null;
  private DigitalInput intakeIRSensor;
  private DigitalInput clawIRSensor;
  public static final double NAVX_ERROR_CODE = -40001;


  public MustangSensors(){
    try {
      navXMicro = new NavX(RobotMap.NAVX_PORT); 
    } catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
			navXMicro = null;
    } 
    
    try {
      intakeIRSensor = new DigitalInput(RobotMap.INTAKE_IR_DIO_PORT);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating intakeIRSensor: " + ex.getMessage(), true);
      intakeIRSensor = null;
    }

    try {
      clawIRSensor = new DigitalInput(RobotMap.CLAW_IR_DIO_PORT);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating clawIRSensor: " + ex.getMessage(), true);
      clawIRSensor = null;
    }
  }

  @Override
  public void initDefaultCommand() {
    // No Default Command
  }

  /**
   * Resets the NavX. This means that the NavX performs a shut-down and recalibration, taking time. Use this if it significantly drifs.
   */
  public void resetNavX() {
		if(navXMicro != null) {
      navXMicro.reset();
    }
	}

  /**
   * Returns the rate of change of the yaw angle in degrees per second. If NavX not connected, returns NAVX_ERROR_CODE. 
   */
  public double getYawRateDegreesPerSecond() {
    if(navXMicro != null) {
      return navXMicro.getYawRateDegreesPerSec();
    } else{
      return NAVX_ERROR_CODE;
    }
  }

  /**
   * Returns the yaw of the robot as a double in accordance with its adjustment based on Robot starting position. (-180, 180)
   * @return The yaw of the robot if the navX is connected. Otherwise returns NAVX_ERROR_CODE
   */
  public double getYawDouble(){
    if(navXMicro != null) {
      return navXMicro.getYawDouble();
    } else{
      return NAVX_ERROR_CODE;
    }
  }

  public double getPitchDouble() {
    if(navXMicro != null) {
      return navXMicro.getPitch();
    } else{
      return NAVX_ERROR_CODE;
    }
  }

  /**
   * 
   * Gets the yaw for Pathfinder since it needs it mirrored from the normal way. (180, -180). If NavX not connected, returns NAVX_ERROR_CODE
   * 
   * @return Yaw as a double (180, -180)
   */
  public double getYawDoubleForPathfinder(){
   return -1 * getYawDouble();
  }

  public Rotation getRotationAngle() {
    double headingRadians = Math.toRadians(getYawDouble());
    return new Rotation(Math.cos(headingRadians), Math.sin(headingRadians));
  }

  /**
   * Gets the yaw as one of 254's Rotation2d Objects (a point on the unit circle). Returns null if the navX is not connected.
   */
  // public Rotation2d getYaw() {
  //   if(navXMicro != null) {
  //   return navXMicro.getYaw();
  //   } else {
  //     return null;
  //   }
  // }

  /**
   * Adds a "user-offset" variable to the NavX, effectively zeroing the value you will receive at this point.
   */
  public void zeroYaw(){
    if(navXMicro != null){
      navXMicro.zeroYaw();
    }
  }

  public double getFieldCentricYaw() {
    if (navXMicro != null) {
      return navXMicro.getYawFieldCentric();
    } else {
      return NAVX_ERROR_CODE;
    }
  }

  public double getAngle() {
    if (navXMicro != null) {
      return navXMicro.getAngle();
    } else {
      return NAVX_ERROR_CODE;
    }
  }

  public ZeroableNavX_Yaw_PIDSource getZeroableNavXPIDSource() {
    if(navXMicro != null){
      return navXMicro.getZeroableNavXYawPIDSource();
    }
    return null;
  }

  /**
   * Returns a PIDSource with the NavX pitch
   */
  public NavX_Pitch_PIDSource getNavXPitchPIDSource() {
    if(navXMicro != null){
      return navXMicro.getNavXPitchPIDSource();
    }
    return null;
  }

  /**
   * Returns true if object is within threshold and false if not
   */
  public boolean getIntakeIROutput(){
    if(intakeIRSensor != null){
      return intakeIRSensor.get();
    }
    return false;
  }


  /**
   * Returns true if object is within threshold and false if not
   */
  public boolean getClawIROutput(){
    if(clawIRSensor != null){
      return clawIRSensor.get();
    }
    return false;
  }

  /**
   * Returns the intake IR sensor
   */
  public DigitalInput getIntakeIRSensor(){
    return intakeIRSensor;
  }

  /**
   * Returns the claw IR sensor
   */
  public DigitalInput getClawIRSensor(){
    return clawIRSensor;
  }

  /**
   * Returns the navX
   */
  public NavX getNavX(){
    return navXMicro;
  }
}