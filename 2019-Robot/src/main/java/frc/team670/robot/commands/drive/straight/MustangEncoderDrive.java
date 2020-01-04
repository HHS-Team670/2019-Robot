/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.straight;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;

public class MustangEncoderDrive extends CommandBase {
  private PIDController leftPIDController;
  private PIDController rightPIDController;
  private int P, I, D, ticksToTravel, tolerance = 300; //FF

  public MustangEncoderDrive(int inchesToTravel) {
    addRequirements(Robot.driveBase);
    ticksToTravel = DriveBase.convertInchesToDriveBaseTicks(inchesToTravel);

    leftPIDController = new PIDController(P, I, D);// FF, Robot.driveBase.getLeftMustangDriveBaseEncoder(), Robot.driveBase.getLeftControllers().get(0));
    rightPIDController = new PIDController(P, I, D);// FF, Robot.driveBase.getRightMustangDriveBaseEncoder(), Robot.driveBase.getRightControllers().get(0));
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    leftPIDController.setIntegratorRange(-1, 1);
    //leftPIDController.setOutputRange(-1, 1);
    rightPIDController.setIntegratorRange(-1, 1);
    //rightPIDController.setOutputRange(-1, 1);

    leftPIDController.setTolerance(tolerance);
    rightPIDController.setTolerance(tolerance);

    leftPIDController.setSetpoint(ticksToTravel);
    rightPIDController.setSetpoint(ticksToTravel);

    Logger.consoleLog("leftStartingPosition:%s rightStartingPosition:%s ", Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks());

    // leftPIDController.enable();
    // rightPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Logger.consoleLog("leftCurrentPosition:%s rightCurrentPosition:%s ", Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (leftPIDController.atSetpoint() && rightPIDController.atSetpoint());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Robot.driveBase.stop();

    Logger.consoleLog("leftEndingPosition:%s rightEndingPosition:%s ", Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks());
  }
/*
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
    end();
    Logger.consoleLog("Interrupted");
  }
  */
}