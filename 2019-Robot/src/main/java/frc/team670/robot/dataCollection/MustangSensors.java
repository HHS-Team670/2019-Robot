package frc.team670.robot.dataCollection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
  public static final double ULTRASONIC_ERROR_CODE = 99999;

  private static final double FRONT_ULTRA_OFFSET = -12.25, BACK_LEFT_ULTRA_OFFSET = -9.25, BACK_RIGHT_ULTRA_OFFSET = 9.25;

   //Ultrasonic
   private DIOUltrasonic frontUltrasonic, backLeftUltrasonic, backRightUltrasonic;


  public MustangSensors(){
    try {
      navXMicro = new NavX(RobotMap.NAVX_PORT); 
      isNavXNull = false;
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      SmartDashboard.putString("warning", "Error instantiating navX-MXP");
      navXMicro = null;
      isNavXNull = true;
    }

    try {
      intakeIRSensor = new DigitalInput(RobotMap.INTAKE_IR_DIO_PORT);
    } catch (RuntimeException ex) {
      // DriverStation.reportError("Error instantiating intakeIRSensor: " + ex.getMessage(), true);
      // SmartDashboard.putString("warning", "Error instantiating intakeIRSensor");
      intakeIRSensor = null;
    }

    try {
      frontUltrasonic = new DIOUltrasonic(RobotMap.FRONT_ULTRASONIC_TRIGGER_PIN, RobotMap.FRONT_ULTRASONIC_ECHO_PIN, FRONT_ULTRA_OFFSET);
    } catch (RuntimeException ex) {
      // DriverStation.reportError("Error instantiating front ultrasonic: " + ex.getMessage(), true);
      // SmartDashboard.putString("warning", "Error instantiating front ultrasonic");
      frontUltrasonic = null;
    }

    try {
      backLeftUltrasonic= new DIOUltrasonic(RobotMap.BACK_LEFT_ULTRASONIC_TRIGGER_PIN, RobotMap.BACK_LEFT_ULTRASONIC_ECHO_PIN, BACK_LEFT_ULTRA_OFFSET);
    } catch (RuntimeException ex) {
      // DriverStation.reportError("Error instantiating back left ultrasonic: " + ex.getMessage(), true);
      // SmartDashboard.putString("warning", "Error instantiating back left ultrasonic");
      backLeftUltrasonic = null;
    }

    try {
      backRightUltrasonic = new DIOUltrasonic(RobotMap.BACK_RIGHT_ULTRASONIC_TRIGGER_PIN, RobotMap.BACK_RIGHT_ULTRASONIC_ECHO_PIN, BACK_RIGHT_ULTRA_OFFSET);
    } catch (RuntimeException ex) {
      // DriverStation.reportError("Error instantiating back right ultrasonic: " + ex.getMessage(), true);
      // SmartDashboard.putString("warning", "Error instantiating back right ultrasonic");
      backRightUltrasonic = null;
    }
  }

  /*
   * Returns distance as given by ultrasonic
   */
  public double getFrontUltrasonicUnadjustedDistance(){
    if(frontUltrasonic != null)
      return frontUltrasonic.getUnadjustedDistance();
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /*
   * Returns adjusted distance as given by ultrasonic
   */
  public double getFrontUltrasonicDistance(double angle){
    if(frontUltrasonic != null)
      return frontUltrasonic.getDistance(angle);
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /*
   * Returns distance as given by ultrasonic
   */
  public double getBackRightUltrasonicUnadjustedDistance(){
    if(backRightUltrasonic != null)
      return backRightUltrasonic.getUnadjustedDistance();
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /*
   * Returns adjusted distance as given by ultrasonic
   */
  public double getBackRightUltrasonicDistance(double angle){
    if(backRightUltrasonic != null)
      return backRightUltrasonic.getDistance(angle);
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /*
   * Returns distance as given by ultrasonic
   */
  public double getBackLeftUltrasonicUnadjustedDistance(){
    if(backLeftUltrasonic != null)
      return backLeftUltrasonic.getUnadjustedDistance();
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /*
   * Returns adjusted distance as given by ultrasonic
   */
  public double getBackLeftUltrasonicDistance(double angle){
    if(backLeftUltrasonic != null)
      return backLeftUltrasonic.getDistance(angle);
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /**
   * Gets the Rotation for the Pure Pursuit drive. (-180, 180) with 90 being forward
   */
  public Rotation getRotationAngle() {
    double headingRadians = Pathfinder.boundHalfDegrees(90 - getYawDouble());
    return Rotation.fromDegrees(headingRadians);
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


  /*
   * Gets the angle needed to turn to the nearest target - based on navx fieldcentric angle and fieldcentric angle of the
   * target to determine which target you're looking at realistically - uses this for offset calculations on camera/ultrasonic
   */
  public double getAngleToTarget(){
    double target_angle = 0;
    double fieldCentricAngle = getAngle() % 360;

    //Rocket 1 - Right
    if (fieldCentricAngle >= 15.9 && fieldCentricAngle <= 41.9) {
      target_angle = 28.9;
    }
    //Rocket 3 - Right
    else if (fieldCentricAngle >= 138.1 && fieldCentricAngle <= 164.1) {
      target_angle = 151.1;
    }
    //Rocket 3 - Left
    else if (fieldCentricAngle >= 195.9 && fieldCentricAngle <= 221.9) {
      target_angle = 208.9;
    }
    //Rocket 1 - Left 
    else if (fieldCentricAngle >= 318.1 && fieldCentricAngle <= 344.1) {
      target_angle = 331.1;
    }

    // Cargo Ship side targets
    else if (fieldCentricAngle >= 257 && fieldCentricAngle <= 283) {
      target_angle = 270;
    } else if (fieldCentricAngle >= 77 && fieldCentricAngle <= 103) {
      target_angle = 90;
    }

    // Exchange 
    else if (fieldCentricAngle >= 167 && fieldCentricAngle <= 193) {
      target_angle = 180;
    }

    // Front cargo ship
    else if(fieldCentricAngle >= 347 && fieldCentricAngle <= 13){
      target_angle = 0;
    }

    return target_angle - fieldCentricAngle;
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

  public void sendUltrasonicDataToDashboard(){
    if (frontUltrasonic != null && backLeftUltrasonic != null  && backRightUltrasonic != null) {
      SmartDashboard.putNumber("Front Ultrasonic: ", frontUltrasonic.getUnadjustedDistance());
      SmartDashboard.putNumber("Back Left Ultrasonic: ", backLeftUltrasonic.getUnadjustedDistance());
      SmartDashboard.putNumber("Back Right Ultrasonic: ", backRightUltrasonic.getUnadjustedDistance());
    } 
  
     
    if (frontUltrasonic == null) {
      SmartDashboard.putString("Front Ultrasonic: ", "FRONT ULTRASONIC IS NULL!");
    } 
    if (backLeftUltrasonic == null) {
      SmartDashboard.putString("Back Left Ultrasonic: ", "BACK LEFT ULTRASONIC IS NULL!");
    }

    if (backRightUltrasonic == null) {
      SmartDashboard.putString("Back RIGHT Ultrasonic: ", "BACK RIGHT ULTRASONIC IS NULL!");
    }
  }
}