/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;


public class LinearPistonClimb extends Command {

  public LinearPistonClimb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climber.enableClimberPIDControllers(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double frontPower = 0.5;
    double backPower = 0.5;
    if (Robot.sensors.getPitchDouble() > 5) { // i'm assuming this means tilted backwards
      frontPower -= 0.1;
    }
    if (Robot.sensors.getPitchDouble() < 5) { // assuming this means tilted forwards
      backPower -= 0.1;
    }
    Robot.climber.drivePistons(frontPower, backPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climber.getBackPistonsRetracted() && Robot.climber.getBackController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.drivePistons(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MINIMUM_PISTON_POWER);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
