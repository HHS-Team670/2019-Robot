/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.teleop;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.Robot;

/**
 * Flips the direction of the drive: forward or reversed.
 */
public class ToggleDriveSafe extends InstantCommand {

  public ToggleDriveSafe() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    boolean isChildSafe = XboxRocketLeagueDrive.isChildSafe();
    
    XboxRocketLeagueDrive.setDriveReversed(!isChildSafe);
    Logger.consoleLog("Toggled ChildSafeMode: %s", (!isChildSafe));
  }

}
