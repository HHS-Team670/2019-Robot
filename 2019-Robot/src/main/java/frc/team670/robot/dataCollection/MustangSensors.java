package frc.team670.robot.dataCollection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.NavX.NavX_Pitch_PIDSource;
import frc.team670.robot.dataCollection.NavX.ZeroableNavX_Yaw_PIDSource;

/**
 * Instantiates sensor representation objects and contains methods for accessing the sensor data.
 * @author shaylandias
 */
public class MustangSensors extends Subsystem {

  // NavX
  private NavX navXMicro = null;
  public static final double NAVX_ERROR_CODE = -40001;


  public MustangSensors(){
    try {
			navXMicro = new NavX(RobotMap.NAVX_PORT);
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
    return navXMicro.getYawFieldCentric();
  }

  public ZeroableNavX_Yaw_PIDSource getZeroableNavXPIDSource() {
    if(navXMicro != null){
      return navXMicro.getZeroableNavXYawPIDSource();
    }
    return null;
  }

  public NavX_Pitch_PIDSource getNavXPitchPIDSource() {
    if(navXMicro != null){
      return navXMicro.getNavXPitchPIDSource();
    }
    return null;
  }

}