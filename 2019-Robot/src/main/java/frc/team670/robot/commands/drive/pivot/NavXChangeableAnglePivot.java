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
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MutableDouble;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.Robot;
import jaci.pathfinder.Pathfinder;

public class NavXChangeableAnglePivot extends Command {

  private DriveBase driveBase;
  private MustangSensors sensors;
  private MutableDouble angle;
  private double startAngle, finalAngle;
  private PIDController pivotController;
  private boolean reversed;

  private static final double P = 0.0165, I = 0.0, D = 0.0; //0.2
  private static final double ABSOLUTE_TOLERANCE = 2;

  private int onTargetCount;


  public NavXChangeableAnglePivot(MutableDouble angle, DriveBase driveBase, MustangSensors sensors, boolean reversed) {
    requires(driveBase);
    this.driveBase = driveBase;
    this.sensors = sensors;
    this.angle = angle;
    this.reversed = reversed;

    pivotController = new PIDController(P, I, D,sensors.getZeroableNavXPIDSource(), new NullPIDOutput());

    pivotController.setInputRange(-180, 180);
    pivotController.setOutputRange(-0.17, 0.17);
    pivotController.setAbsoluteTolerance(ABSOLUTE_TOLERANCE);
    pivotController.setContinuous(true);
    setTimeout(1.6);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // sensors.zeroYaw();
    System.out.println(sensors.getYawDouble());
    onTargetCount = 0;
    if(sensors.isNavXNull()){
      // cancel();
      finalAngle = startAngle;
      System.out.println("Null N avX");
      return;
    }

    if(Math.abs(angle.getValue()) < 1) {
      // cancel();
      finalAngle = startAngle;
      System.out.println("Angle less than one");
      return;
    }

    if(MathUtils.doublesEqual(RobotConstants.VISION_ERROR_CODE, -Math.abs(angle.getValue()))) {
      finalAngle = startAngle;
      System.out.println("Invalid Vision Data");
      return;
    }

    startAngle = sensors.getYawDouble();
    finalAngle = Pathfinder.boundHalfDegrees(startAngle + angle.getValue());

    System.out.println("StartAngle: " + startAngle + ", degreestotravelto:" + angle.getValue());

    Logger.consoleLog("StartAngle:%s FinalAngle:%s DegreesToTravel:%s", startAngle, finalAngle, angle);

    pivotController.setSetpoint(finalAngle);

    pivotController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double output = pivotController.get();
    // output += output < 0 ? -0.03 : 0.03; // Arbitrary feedforward
	  System.out.println("Output: " + output);
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
    return pivotController.onTarget() || isTimedOut();
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
