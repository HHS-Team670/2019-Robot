package frc.team670.robot.dataCollection;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
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

  public double getYawDouble(){
    if(navXMicro != null) {
      return navXMicro.getYaw().getDegrees();
    } else{
      return NAVX_ERROR_CODE;
    }
  }

  public Rotation2d getYaw() {
    if(navXMicro != null) {
    return navXMicro.getYaw();
    } else {
      return null;
    }
  }

  public void zeroYaw(){
    if(navXMicro != null){
      navXMicro.zeroYaw();
    }
  }

}