/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

public class SparkCANPIDEncoderDrive extends Command {

  private int inchesToTravel, ticksToTravel;
  private double rotations;
  private int leftStartingPosition, rightStartingPosition, leftEndingPosition, rightEndingPosition, leftCurrentPosition, rightCurrentPosition;
  private double threshold = 300; // TODO Define threshold

  public SparkCANPIDEncoderDrive(int inchesToTravel) {
    requires (Robot.driveBase);
    this.inchesToTravel = inchesToTravel;

    ticksToTravel = MathUtils.convertInchesToDriveBaseTicks(inchesToTravel);

    rotations = ticksToTravel / RobotConstants.SPARK_TICKS_PER_ROTATION;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftStartingPosition = Robot.driveBase.getLeftSparkEncoderPosition();
    rightStartingPosition = Robot.driveBase.getRightSparkEncoderPosition();

    Logger.consoleLog("leftStartingPosition:%s rightStartingPosition:%s ", leftStartingPosition, rightStartingPosition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    leftCurrentPosition = Robot.driveBase.getLeftSparkEncoderPosition();
    rightCurrentPosition = Robot.driveBase.getRightSparkEncoderPosition();

    Robot.driveBase.setSparkEncodersControl(ticksToTravel, ticksToTravel); // Could be put into initialize
    
    // Also possibly takes in rotations not tick values
    Logger.consoleLog("lefCurrentPosition:%s rightCurrentPosition:%s ", leftCurrentPosition, rightCurrentPosition);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if (Math.abs(Robot.driveBase.getLeftSparkEncoderPosition() - (ticksToTravel + leftStartingPosition)) <= threshold &&
     Math.abs(Robot.driveBase.getRightSparkEncoderPosition() - (ticksToTravel + rightStartingPosition)) <= threshold ){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.stop();

    leftEndingPosition = Robot.driveBase.getLeftSparkEncoderPosition();
    rightEndingPosition = Robot.driveBase.getRightSparkEncoderPosition();

    Logger.consoleLog("leftEndingPosition:%s rightEndingPosition:%s ", leftEndingPosition, rightEndingPosition);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveBase.stop();

    Logger.consoleLog("CANPIDEncoderDrive interrupted");
  }
}
