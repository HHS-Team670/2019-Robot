/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.straight;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;

public class MustangEncoderDrive extends Command {
  private PIDController leftPIDController;
  private PIDController rightPIDController;
  private int P, I, D, FF, ticksToTravel, tolerance = 300;

  public MustangEncoderDrive(int inchesToTravel) {
    requires(Robot.driveBase);
    ticksToTravel = DriveBase.convertInchesToDriveBaseTicks(inchesToTravel);

    leftPIDController = new PIDController(P, I, D, FF, Robot.driveBase.getLeftMustangDriveBaseEncoder(), Robot.driveBase.getLeftControllers().get(0));
    rightPIDController = new PIDController(P, I, D, FF, Robot.driveBase.getRightMustangDriveBaseEncoder(), Robot.driveBase.getRightControllers().get(0));
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftPIDController.setOutputRange(-1, 1);
    rightPIDController.setOutputRange(-1, 1);

    leftPIDController.setAbsoluteTolerance(tolerance);
    rightPIDController.setAbsoluteTolerance(tolerance);

    leftPIDController.setSetpoint(ticksToTravel);
    rightPIDController.setSetpoint(ticksToTravel);

    Logger.consoleLog("leftStartingPosition:%s rightStartingPosition:%s ", Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks());

    leftPIDController.enable();
    rightPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Logger.consoleLog("leftCurrentPosition:%s rightCurrentPosition:%s ", Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (leftPIDController.onTarget() && rightPIDController.onTarget());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.stop();

    Logger.consoleLog("leftEndingPosition:%s rightEndingPosition:%s ", Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog("Interrupted");
  }
}