/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.straight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.Logger;

/**
 * An example command.  You can replace me with your own command.
 */
public class TimeDrive extends CommandBase {

    private double speed, seconds;
    private int executeCounter;

  public TimeDrive(double seconds, double speed) {
    this.speed = speed;
    this.seconds = seconds;
    executeCounter = 0;
    addRequirements(Robot.driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    set(seconds);
    Logger.consoleLog("Speed: %s Seconds: %s", speed, seconds);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() { 
    Robot.driveBase.tankDrive(speed, speed, false);
    Logger.consoleLog();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Robot.driveBase.stop();
    Logger.consoleLog("Speed: %s Seconds: %s", speed, seconds);
  }

/*
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
    end();
  }
  */
}
