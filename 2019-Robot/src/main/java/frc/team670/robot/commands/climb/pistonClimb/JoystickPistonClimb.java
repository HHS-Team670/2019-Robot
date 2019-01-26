/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.Logger;


/**
 * Runs the piston climb with a percent output based on input from the joystick
 * 
 */
public class JoystickPistonClimb extends Command {
  private Climber climber;
  private MustangController controller;
  private int tolerance = 500 + RobotConstants.PISTON_ENCODER_FLAT;
  private int loggingIterationCounter;

  /**
   * @param climber the climber being used
   * @param controller the operator controller being used
   */
  public JoystickPistonClimb(Climber climber, MustangController controller) {
    this.climber = climber;
    this.controller = controller;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog("startBackPistonPosition:%s startFrontPistonPosition:%s ", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double frontPower = controller.getLeftStickY() * Climber.MAXIMUM_PISTON_POWER;
    double backPower = controller.getRightStickY() * Climber.MINIMUM_PISTON_POWER;

    if(climber.getFrontTalonPositionInTicks() <= RobotConstants.PISTON_ENCODER_FLAT + tolerance){
      frontPower *= 0.5;
    }

    if(climber.getBackTalonPositionInTicks() <= RobotConstants.PISTON_ENCODER_FLAT + tolerance){
      backPower *= 0.5;
    }

    if(climber.getFrontTalonPositionInTicks() >= RobotConstants.PISTON_ENCODER_LEVEL_THREE - tolerance){
      frontPower *= 0.5;
    }

    if(climber.getBackTalonPositionInTicks() >= RobotConstants.PISTON_ENCODER_LEVEL_THREE - tolerance){
      backPower *= 0.5;
    }


    climber.drivePistons(frontPower, backPower);

    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("currentBackPistonPosition:%s currentFrontPistonPosition:%s", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.drivePistons(0, 0);
    Logger.consoleLog("endBackPistonPosition:%s endFrontPistonPosition:%s ", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    climber.drivePistons(0, 0);

    Logger.consoleLog("JoystickPistonClimb interrupted");
  }
}
