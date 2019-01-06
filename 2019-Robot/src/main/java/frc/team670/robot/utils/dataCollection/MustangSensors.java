package frc.team670.robot.utils.dataCollection;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;

/**
 * Instantiates sensor representation objects and contains methods for accessing the sensor data.
 */
public class MustangSensors extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // NavX
  private AHRS navXMicro = null;
  private boolean isNavXConnected;

  // IR Sensors
  private static DigitalInput dio, dio1;


  public MustangSensors(){

    try {
			navXMicro = new AHRS(RobotMap.navXPort);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
			navXMicro = null;
		}
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  /**
   * Resets the NavX Angle
   */
  public void resetNavX() {
		if(navXMicro != null)
      navXMicro.reset();
	}

	/**
	 * 
	 * Gets a 0-360 bound degree angle of the robot
	 * 
	 * @return 0-360 Degree Angle of Bot
	 */
	public double getAngle()
	{
		if(navXMicro != null){
      double angle = navXMicro.getAngle() % 360;
      if(angle < 0){
        angle += 360;
      }
      return angle;
    }
		else
			return -1;
  }
  
  /**
   * Gets the NavX Angle unbound, can go over 360 degrees
   */
  public double getRawAngle(){
    if(navXMicro != null){
      double angle = navXMicro.getAngle();
      return angle;
    }
		else
			return -1;
  }

  public AHRS getNavX(){
    return navXMicro;
  }

  public double getYaw(){
    if(navXMicro != null){
      double angle = navXMicro.getYaw();
      return angle;
    }
		else
			return -1;
  }

  public void zeroYaw(){
    if(navXMicro != null){
      navXMicro.zeroYaw();
    }
  }
	
	public boolean isNavXConnected() {
		if(navXMicro != null)
			return navXMicro.isConnected();
		return false;
  }
  
  /**
   * Returns true if one IR Sensor is triggered
   */
  public static boolean getIntakeLimit() {
		return (!dio.get() || !dio1.get());	
	}

}