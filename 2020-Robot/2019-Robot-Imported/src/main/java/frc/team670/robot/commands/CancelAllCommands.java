/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.DriveBase;

public class CancelAllCommands extends InstantCommand {

  public CancelAllCommands(DriveBase driveBase, Arm arm, BaseIntake intake, Claw claw) {
    requires(driveBase);
    requires(arm.getElbow());
    requires(arm.getExtension());
    requires(arm.getWrist());
    requires(intake);
    requires(claw);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "CancelAllCommands");
  }

}
