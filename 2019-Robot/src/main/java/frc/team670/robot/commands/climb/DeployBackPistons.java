/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.functions.SettingUtils;

public class DeployBackPistons extends Command {

  public DeployBackPistons() {
    if(!Robot.climber.getFrontPistonsRetracted()){
      this.cancel();
    }

    Robot.climber.setBackPistonsRetracted(false);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climber.getBackController().setSetpoint(RobotConstants.PISTON_ENCODER_FLAT);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climber.getBackController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SettingUtils.releaseController(Robot.climber.getBackController());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    SettingUtils.releaseController(Robot.climber.getBackController());
  }
}
