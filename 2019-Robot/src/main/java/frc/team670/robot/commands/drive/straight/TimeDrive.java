/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.straight;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.Logger;

/**
 * An example command.  You can replace me with your own command.
 */
public class TimeDrive extends Command {

    private double speed, seconds;
    private int executeCounter;

  public TimeDrive(double seconds, double speed) {
    this.speed = speed;
    this.seconds = seconds;
    executeCounter = 0;
    requires(Robot.driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(seconds);
    // Robot.driveBase.LogLeftMotorInfo();
    // Robot.driveBase.LogRightMotorInfo();

    Robot.driveBase.tankDrive(speed, speed, false);
    //Logger.consoleLog("Speed: %s Seconds: %s", speed, seconds);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    Robot.driveBase.feedWatchDog();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.stop();
    Logger.consoleLog("Speed: %s Seconds: %s", speed, seconds);
    // Robot.driveBase.LogLeftMotorInfo();
    // Robot.driveBase.LogRightMotorInfo();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
