package frc.team670.robot.dataCollection;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team254.lib.util.drivers.NavX;
import frc.team254.lib.util.math.Rotation2d;
import frc.team670.robot.constants.RobotMap;


/**
 * Instantiates sensor representation objects and contains methods for accessing the sensor data.
 */
public class MustangSensors extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // NavX
  private NavX navXMicro = null;
  public static final double NAVX_ERROR_CODE = -40001;


  public MustangSensors(){

    try {
			navXMicro = new NavX(RobotMap.navXPort);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
			navXMicro = null;
		}
  }

  @Override
  public void initDefaultCommand() {
    // No Default Command
  }

  /**
   * Resets the NavX Angle
   */
  public void resetNavX() {
		if(navXMicro != null) {
      navXMicro.reset();
    }
	}

  /**
   * Returns the rate of change of the yaw angle in degrees per second.
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
      return navXMicro.getYaw().getDegrees();
    } else{
      return NAVX_ERROR_CODE;
    }
  }

  /**
   * 
   * Gets the yaw for Pathfinder since it needs it mirrored from the normal way. (180, -180)
   * 
   * @return Yaw as a double (180, -180)
   */
  public double getYawDoubleForPathfinder(){
   return -1 * getYawDouble();
  }

  /**
   * Gets the yaw as one of 254's Rotation2d Objects (a point on the unit circle).
   */
  public Rotation2d getYaw() {
    if(navXMicro != null) {
    return navXMicro.getYaw();
    } else {
      return null;
    }
  }



  /**
   * Resets the yaw angle to zero and the acceleration of the angle to zero.
   */
  public void zeroYaw(){
    if(navXMicro != null){
      navXMicro.zeroYaw();
    }
  }
  
  
}