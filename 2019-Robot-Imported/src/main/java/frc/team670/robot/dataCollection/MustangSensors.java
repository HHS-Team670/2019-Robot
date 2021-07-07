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
  private DigitalInput intakeBeamBreak;
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
      intakeBeamBreak = new DigitalInput(RobotMap.INTAKE_BEAM_BREAK_DIO_PORT);
    } catch (RuntimeException ex) {
      // DriverStation.reportError("Error instantiating intakeIRSensor: " + ex.getMessage(), true);
      // SmartDashboard.putString("warning", "Error instantiating intakeIRSensor");
      intakeBeamBreak = null;
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

    frontUltrasonic.setUltrasonicAutomaticMode(true); // This will set it for all the sensors. WPI should definitely have made this method static.
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
   * Returns adjusted distance as given by ultrasonic - accounts for offset
   */
  public double getFrontUltrasonicDistance(){
    if(frontUltrasonic != null)
      return frontUltrasonic.getDistance();
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
   * Returns adjusted distance as given by ultrasonic - accounts for offset
   */
  public double getBackRightUltrasonicDistance(){
    if(backRightUltrasonic != null)
      return backRightUltrasonic.getDistance();
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
   * Returns adjusted distance as given by ultrasonic - accounts for offset
   */
  public double getBackLeftUltrasonicDistance(){
    if(backLeftUltrasonic != null)
      return backLeftUltrasonic.getDistance();
    else
      return ULTRASONIC_ERROR_CODE;
  }

  /*
   * Returns the adjusted distance, taking into account both the back ultrasonics
   * If both ultrasonics have similar data it averages; if only one has "good" data it uses the adjusted distance
   */
  public double getAdjustedBackUltrasonicDistance(){
    double backLeft = getBackLeftUltrasonicUnadjustedDistance();
    double backRight = getBackRightUltrasonicUnadjustedDistance();
    double adjustedDistance = (backLeft < backRight) ? backLeft : backRight; // Take the one that is least;

    // Below check if both sensors are getting valid data, if not pick the one that seems right, or return the error code
    if(backLeft >= 150 && backRight >= 150) {
      return ULTRASONIC_ERROR_CODE;
    }
    else if(backRight < 150 && backLeft > 150){
      return backRight;
    }
    else if(backLeft < 150 && backRight > 150){
      return backLeft;
    }

    if(Math.abs(backRight-backLeft) <= 10){
      adjustedDistance = (backLeft+backRight)/2.0; //average both if they're essentially the same
    }

    return adjustedDistance;
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

  /**
   * Gets fieldcentric angle of navX 
   */
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
    if (fieldCentricAngle > 14 && fieldCentricAngle <= 65) {
      target_angle = 28.9;
    }
    //Rocket 3 - Right
    else if (fieldCentricAngle > 115 && fieldCentricAngle <= 167) {
      target_angle = 151.1;
    }
    //Rocket 3 - Left
    else if (fieldCentricAngle > 194 && fieldCentricAngle <= 255) {
      target_angle = 208.9;
    }
    //Rocket 1 - Left 
    else if (fieldCentricAngle > 285 && fieldCentricAngle <= 347) {
      target_angle = 331.1;
    }

    // Cargo Ship side targets
    else if (fieldCentricAngle > 255 && fieldCentricAngle <= 285) {
      target_angle = 270;
    } else if (fieldCentricAngle > 65 && fieldCentricAngle <= 115) {
      target_angle = 90;
    }

    // Exchange 
    else if (fieldCentricAngle > 167 && fieldCentricAngle <= 194) {
      target_angle = 180;
    }

    // Front cargo ship
    else if(fieldCentricAngle > 347 || fieldCentricAngle <= 14){
      target_angle = 0;
    }

    SmartDashboard.putNumber("Target Angle", target_angle);

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
      return !intakeIRSensor.get(); // .get() Returns true if triggered, false if not
    }
    return false;
  }

  public boolean getIntakeBeamBreakOutput(){
    if(intakeBeamBreak != null){
      return intakeBeamBreak.get(); // .get() Returns true if triggered, false if not
    }
    return false;
  }


  /**
   * Returns the intake IR sensor
   */
  public boolean isIntakeIRSensorNull(){
    return intakeIRSensor == null;
  }


  public boolean isIntakeBeamBreakNull(){
    return intakeBeamBreak == null;
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
    if (frontUltrasonic != null){
      SmartDashboard.putNumber("Front Ultrasonic: ", frontUltrasonic.getUnadjustedDistance());
    } else if (frontUltrasonic == null) {
      SmartDashboard.putString("Front Ultrasonic: ", "FRONT ULTRASONIC IS NULL!");
    }

    if(backLeftUltrasonic != null){
      SmartDashboard.putNumber("Back Left Ultrasonic: ", backLeftUltrasonic.getUnadjustedDistance());
    } else if (backLeftUltrasonic == null) {
      SmartDashboard.putString("Back Left Ultrasonic: ", "BACK LEFT ULTRASONIC IS NULL!");
    }

    if(backRightUltrasonic != null) { 
      SmartDashboard.putNumber("Back Right Ultrasonic: ", backRightUltrasonic.getUnadjustedDistance());
    } else if (backRightUltrasonic == null) {
      SmartDashboard.putString("Back Right Ultrasonic: ", "BACK RIGHT ULTRASONIC IS NULL!");
    }
  }

  public void sendBreamBreakDataToDashboard() {
    if(!isIntakeBeamBreakNull()) {
      // SmartDashboard.putString("Intake IR", getIntakeIROutput() + "");
      SmartDashboard.putString("Intake Beam Break", getIntakeBeamBreakOutput() + "");
    }
    else {
      SmartDashboard.putString("Intake Beam Break", "null");
    }
  }

}