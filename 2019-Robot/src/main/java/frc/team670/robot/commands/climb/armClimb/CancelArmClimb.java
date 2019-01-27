/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.armClimb;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm;

/**
 * Turns a boolean in the ArmClimb class to false which stops it from running
 */
public class CancelArmClimb extends InstantCommand {

  public CancelArmClimb(Arm arm) {
    super();
    requires(arm.getElbow());
    requires(arm.getWrist());
    requires(arm.getExtension());
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    ArmClimb.setUserWishesToStillClimb(false);
  }

}
