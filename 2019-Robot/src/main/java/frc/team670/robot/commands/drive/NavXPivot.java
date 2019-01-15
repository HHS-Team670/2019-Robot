/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.Logger;

/**
 * An example command.  You can replace me with your own command.
 */
public class NavXPivot extends Command {

	private double finalAngle, startAngle, angle, leftSpeed, rightSpeed;
	protected double endingSpeed = 0.2;
	private PIDController pivotController;
	private static final double P = 1, I = 0, D = 0;

  public NavXPivot(double angle) {
	this.angle = angle;
	

	// TODO find PID constants
	pivotController = new PIDController(P, I, D, Robot.sensors.getZeroableNavXPIDSource(), null);

	pivotController.setInputRange(-180, 180);
	pivotController.setOutputRange(-1, 1);
	pivotController.setAbsoluteTolerance(5);
	pivotController.setContinuous(true);

	requires(Robot.driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		startAngle = Robot.sensors.getYawDouble();
		finalAngle = startAngle + angle;

		Logger.consoleLog("StartAngle:%s FinalAngle:%s DegreesToTravel:%s", 
				startAngle, finalAngle, finalAngle - startAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

		double pivotOutput = pivotController.get();
		Robot.driveBase.tankDrive(leftSpeed, -rightSpeed);
	
	Logger.consoleLog("DegreesToTravel:%s CurrentAngle:%s", 
			yawRemaining(), getNormalizedYaw());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
		return pivotController.onTarget();
  }


  // Called once after isFinished returns true
  @Override
  protected void end() {
		Robot.driveBase.tankDrive(0, 0);
		Logger.consoleLog("DegreesToTravel:%s CurrentAngle:%s", 
				yawRemaining(), getNormalizedYaw());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
		end();
	}
	
		//Gets yaw accounting for discontinuity at 180/-180 for NavX yaw result
		private double getNormalizedYaw() {
			double rawYaw = Robot.sensors.getYawDouble();
			if((startAngle < 0 && rawYaw > 0) || (startAngle > 0 && rawYaw < 0)) { //Signs of angles are different
				if(angle < 0) {
					if(startAngle > 0) {
						return rawYaw;
					} else {
						return -360 + rawYaw; // -180 - (180 - rawYaw)
					}
				}
				else {
					if(startAngle > 0) {
						return 360 + rawYaw; // 180 + (180 + rawYaw)
					} else {
						return rawYaw;
					}
				}
			}
			else {
				return rawYaw;
			}
		}
	
		private double yawRemaining() {
			
			return finalAngle - getNormalizedYaw();
		}
}
