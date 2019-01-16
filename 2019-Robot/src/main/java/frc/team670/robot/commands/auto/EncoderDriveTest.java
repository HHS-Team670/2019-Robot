/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.Logger;

public class EncoderDriveTest extends Command {
  private CANPIDController  leftController, rightController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0; //TODO Set the F value
  private static final double distanceTolerance = 0.05; //inches
  private int startLeftEncoderPosition, startRightEncoderPosition;
  private int executeCount;
  private int inches, rotations, ticksToTravel;

  public EncoderDriveTest(int inches) {
    super();
    requires(Robot.driveBase);

    this.inches = inches;
    startLeftEncoderPosition = Robot.driveBase.getLeftEncoderPosition();
    startRightEncoderPosition = Robot.driveBase.getRightEncoderPosition();
    
    // ticksToTravel = ((inches) / (Math.PI * RobotConstants.DRIVEBASE_WHEEL_DIAMETER))
    // 		* RobotConstants.DRIVEBASE_TICKS_PER_ROTATION;
    // rotations = ticksToTravel/RobotConstants.DRIVEBASE_TICKS_PER_ROTATION;

    leftController = new CANPIDController(Robot.driveBase.getLeft1());
    rightController = new CANPIDController(Robot.driveBase.getRight1());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftController.setP(P);
    leftController.setI(I);
    leftController.setD(D);
    leftController.setFF(F);
    leftController.setOutputRange(-1, 1);

    rightController.setP(P);
    rightController.setI(I);
    rightController.setD(D);
    rightController.setFF(F);
    rightController.setOutputRange(-1, 1);

    Logger.consoleLog("StartLeftEncoder:%s StartRightEncoder:%s", startLeftEncoderPosition, startRightEncoderPosition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveBase.setEncodersControl(rotations, rotations);

    Logger.consoleLog("CurrentLeftEncoder:%s CurrentRightEncoder:%s", Robot.driveBase.getLeftEncoderPosition(), 
    Robot.driveBase.getRightEncoderPosition());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.driveBase.getLeftEncoderPosition() - ticksToTravel) <= distanceTolerance && 
     Math.abs(Robot.driveBase.getRightEncoderPosition() - ticksToTravel) <= distanceTolerance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.stop();
    Logger.consoleLog("FinalLeftEncoder:%s FinalRightEncoder:%s", Robot.driveBase.getLeftEncoderPosition(),
    Robot.driveBase.getRightEncoderPosition());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveBase.stop();
    Logger.consoleLog("Encoder Drive Interrupted");
  }
}
