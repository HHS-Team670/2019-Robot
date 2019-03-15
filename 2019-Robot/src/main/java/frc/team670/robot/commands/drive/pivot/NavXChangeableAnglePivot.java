/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.pivot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MutableDouble;
import frc.team670.robot.Robot;
import jaci.pathfinder.Pathfinder;

public class NavXChangeableAnglePivot extends Command {

  private DriveBase driveBase;
  private MustangSensors sensors;
  private MutableDouble angle;
  private double startAngle, finalAngle;
  private PIDController pivotController;

  private static final double P = 0.05, I = 0.0, D = 0.5;
  private static final double ABSOLUTE_TOLERANCE = 1.5;

  private int onTargetCount;


  public NavXChangeableAnglePivot(MutableDouble angle, DriveBase driveBase, MustangSensors sensors) {
    requires(driveBase);
    this.driveBase = driveBase;
    this.sensors = sensors;
    this.angle = angle;

    pivotController = new PIDController(P, I, D,sensors.getZeroableNavXPIDSource(), new NullPIDOutput());

    pivotController.setInputRange(-180, 180);
    pivotController.setOutputRange(-1, 1);
    pivotController.setAbsoluteTolerance(ABSOLUTE_TOLERANCE);
    pivotController.setContinuous(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println(sensors.getYawDouble());
    onTargetCount = 0;
    if(sensors.isNavXNull()){
      // cancel();
      finalAngle = startAngle;
      return;
    }

    if(angle.getValue() < 3) {
      // cancel();
      finalAngle = startAngle;
      return;
    }

    startAngle = sensors.getYawDouble();
    finalAngle = Pathfinder.boundHalfDegrees(startAngle + angle.getValue());

    Logger.consoleLog("StartAngle:%s FinalAngle:%s DegreesToTravel:%s", startAngle, finalAngle, angle);

    pivotController.setSetpoint(finalAngle);

    pivotController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double output = pivotController.get();
	  // System.out.println("Output: " + output);
	  driveBase.tankDrive(output, -output, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if(pivotController.onTarget()) {
		//   onTargetCount ++;
	  // } else {
		//   onTargetCount = 0;
	  // }
	  // return (onTargetCount > 3);
    return pivotController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveBase.stop();
    System.out.println(sensors.getYawDouble());
    Logger.consoleLog("CurrentAngle: %s, TargetAngle: %s", sensors.getYawDouble(), finalAngle);  
    boolean isReversed = XboxRocketLeagueDrive.isDriveReversed();
    if (isReversed) {
      Robot.leds.setReverseData(true);
    } else {
      Robot.leds.setForwardData(true);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
		Logger.consoleLog();
  }
}
