package frc.team670.robot.dataCollection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.DIOUltrasonic;
import frc.team670.robot.dataCollection.sensors.NavX;
import frc.team670.robot.dataCollection.sensors.NavX.NavX_Pitch_PIDSource;
import frc.team670.robot.dataCollection.sensors.NavX.ZeroableNavX_Yaw_PIDSource;
import frc.team670.robot.utils.math.Rotation;
import jaci.pathfinder.Pathfinder;

/**
 * Instantiates sensor representation objects and contains methods for accessing the sensor data.
 * @author shaylandias
 */
public class MustangSensors {

  // NavX
  private NavX navXMicro = null;
  private boolean isNavXNull;
  private DigitalInput intakeIRSensor;
  public static final double NAVX_ERROR_CODE = -40001;

   //Ultrasonic
   private DIOUltrasonic ultrasonic = null;


  public MustangSensors(){
    try {
      // navXMicro = new NavX(RobotMap.NAVX_PORT); 
      // isNavXNull = false;
    } catch (RuntimeException ex) {
      // DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      // SmartDashboard.putString("sensor-error", "Error instantiating navX-MXP");
      navXMicro = null;
      isNavXNull = true;
    }

    try {
      intakeIRSensor = new DigitalInput(RobotMap.INTAKE_IR_DIO_PORT);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating intakeIRSensor: " + ex.getMessage(), true);
      SmartDashboard.putString("sensor-error", "Error instantiating intakeIRSensor");
      intakeIRSensor = null;
    }

    ultrasonic = new DIOUltrasonic();
  }

  /*
   * Returns distance as given by ultrasonic
   */
  public double getUltrasonicDistance(){
    return ultrasonic.getUltrasonicValue();
  }

  /*
   * Returns ultrasonic object
   */
  public Ultrasonic getUltrasonic(){
    return ultrasonic.getWPIUltrasonicObject();
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
    } else {
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
    } else {
      return NAVX_ERROR_CODE;
    }
  }

  public double getPitchDouble() {
    if(navXMicro != null) {
      return navXMicro.getPitch();
    } else {
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

  /**
   * Gets the Rotation for the Pure Pursuit drive. (-180, 180) with 90 being forward
   */
  public Rotation getRotation() {
    double headingRadians = Pathfinder.boundHalfDegrees(90 - getYawDouble());
    return Rotation.fromDegrees(headingRadians);
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

  /**
   * Returns a PIDSource with the NavX Yaw corresponding to the last zero (not field centric).
   * @return Zeroable NavX Yaw Source, null if the navX could not be instantiated!
   */
  public ZeroableNavX_Yaw_PIDSource getZeroableNavXPIDSource() {
    if(navXMicro != null){
      return navXMicro.getZeroableNavXYawPIDSource();
    }
    return null;
  }

  /**
   * Returns a PIDSource with the NavX pitch.
   * @return NavX Pitch Source, null if the navX could not be instantiated!
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
   * Returns the intake IR sensor
   */
  public boolean isIntakeIRSensorNull(){
    return intakeIRSensor == null;
  }

  /**
   * Returns the navX
   */
  // public NavX getNavX(){
  //   return navXMicro;
  // }

  /**
   * @return true if NavX is null (could not be instantiated), false if you can call methods on the NavX
   */
  public boolean isNavXNull() {
    return isNavXNull;
  }
}