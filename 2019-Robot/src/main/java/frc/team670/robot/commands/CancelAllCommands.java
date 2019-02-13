/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;

public class CancelAllCommands extends InstantCommand {
  public CancelAllCommands() {
    requires(Robot.intake);
    requires(Robot.driveBase);
    requires(Robot.arm.getElbow());
    requires(Robot.arm.getWrist());
    requires(Robot.arm.getExtension());
    requires(Robot.climber);
    requires(Robot.claw);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "CancelAllCommands");
  }

}
