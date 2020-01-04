/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.pivot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.Robot;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.utils.Logger;
import jaci.pathfinder.Pathfinder;

/**
 * An example command.  You can replace me with your own command.
 */
public class NavXPivot extends CommandBase {

	private double finalAngle, startAngle, angle, leftSpeed, rightSpeed;
	protected double endingSpeed = 0.2;
	private PIDController pivotController;
	private static final double P = 0.0055, I = 0.00001, D = 0;

	private int onTargetCount;

  public NavXPivot(double angle) {

	this.angle = angle;
	
	pivotController = new PIDController(P, I, D); //, Robot.sensors.getZeroableNavXPIDSource(), new NullPIDOutput());

	pivotController.enableContinuousInput(-180, 180);
	// pivotController.setInputRange(-180, 180);
	pivotController.setIntegratorRange(-0.17, 0.17);
	// pivotController.setOutputRange(-1, 1);
	pivotController.setTolerance(5);
	// pivotController.setContinuous(true);

	addRequirements(Robot.driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

	if(Robot.sensors.isNavXNull()){
		super.cancel();
		return;
	}

	startAngle = Robot.sensors.getYawDouble();
	finalAngle = Pathfinder.boundHalfDegrees(startAngle + angle);

	// Logger.consoleLog("StartAngle:%s FinalAngle:%s DegreesToTravel:%s", 
	// 		startAngle, finalAngle, angle);

	pivotController.setSetpoint(finalAngle);

	// pivotController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
	
	double output = pivotController.get();
	// System.out.println("Output: " + output);
	Robot.driveBase.tankDrive(output, -output, false);

	// Logger.consoleLog("Output:%s CurrentAngle:%s", output, Robot.sensors.getYawDouble());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
	//   if(pivotController.onTarget()) {
	// 	  onTargetCount ++;
	//   } else {
	// 	  onTargetCount = 0;
	//   }
	// return (onTargetCount > 10);
	return pivotController.atSetpoint();
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
		Robot.driveBase.stop();
		Logger.consoleLog("CurrentAngle: %s, TargetAngle", Robot.sensors.getYawDouble(), finalAngle);
  }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   public void interrupted() {
// 		end();
// 		Logger.consoleLog();
// 	}
	
}
