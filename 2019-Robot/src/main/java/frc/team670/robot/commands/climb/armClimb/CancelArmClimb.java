/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.armClimb;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;

/**
 * Add your docs here.
 */
public class CancelArmClimb extends InstantCommand {

  public CancelArmClimb() {
    super();
    requires(Robot.arm);
    requires(Robot.elbow);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    ArmClimb.canClimb = false;
  }

}
