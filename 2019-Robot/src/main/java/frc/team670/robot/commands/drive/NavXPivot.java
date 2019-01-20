/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.utils.Logger;
import jaci.pathfinder.Pathfinder;

/**
 * An example command.  You can replace me with your own command.
 */
public class NavXPivot extends Command {

	private double finalAngle, startAngle, angle, leftSpeed, rightSpeed;
	protected double endingSpeed = 0.2;
	private PIDController pivotController;
	// TODO find PID constants
	private static final double P = 1, I = 0, D = 0;

  public NavXPivot(double angle) {
	this.angle = angle;
	
	pivotController = new PIDController(P, I, D, Robot.sensors.getZeroableNavXPIDSource(), new NullPIDOutput());

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
		finalAngle = Pathfinder.boundHalfDegrees(startAngle + angle);

		Logger.consoleLog("StartAngle:%s FinalAngle:%s DegreesToTravel:%s", 
				startAngle, finalAngle, angle);

		pivotController.setSetpoint(finalAngle);

		pivotController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
	
	double output = pivotController.get();

	Robot.driveBase.tankDrive(output, -output, false);

	Logger.consoleLog("Output:%s CurrentAngle:%s", output, Robot.sensors.getYawDouble());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
		return pivotController.onTarget();
  }


  // Called once after isFinished returns true
  @Override
  protected void end() {
		Robot.driveBase.stop();
		Logger.consoleLog("FinishedPivot, CurrentAngle: %s, TargetAngle", Robot.sensors.getYawDouble(), finalAngle);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
		end();
		Logger.consoleLog("Pivot Interrupted");
	}
	
}
