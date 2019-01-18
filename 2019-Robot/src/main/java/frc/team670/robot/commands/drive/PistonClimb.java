/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.team254.lib.util.drivers.NavX;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Climber;

public class PistonClimb extends Command {

  public static MustangSensors sensors = new MustangSensors();
  private Climber climber;

  public PistonClimb() {
    climber = new Climber();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double frontPower = 0.5;
    double backPower = 0.5;
    if (sensors.getPitchDouble() > 5) { // i'm assuming this means tilted backwards
      frontPower -= 0.1;
    }
    if (sensors.getPitchDouble() < 5) { // assuming this means tilted forwards
      backPower -= 0.1;
    }
    climber.drivePistons(frontPower, backPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
