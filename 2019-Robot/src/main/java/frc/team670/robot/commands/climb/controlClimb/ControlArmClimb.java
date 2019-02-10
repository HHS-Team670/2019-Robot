/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.controlClimb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.commands.climb.armClimb.ArmClimb;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Climber;

public class ControlArmClimb extends Command {
  private ArmClimb armClimb;
  private Arm arm;
  private Climber climber;

  public ControlArmClimb(Arm arm, Climber climber) {
    armClimb = new ArmClimb(arm, climber);
    this.arm = arm;
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(armClimb.isFinished()){
      armClimb = new ArmClimb(arm, climber);
      Scheduler.getInstance().add(armClimb);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !ArmClimb.getUserWishesToStillClimb();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    armClimb.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    armClimb.cancel();
  }
}
