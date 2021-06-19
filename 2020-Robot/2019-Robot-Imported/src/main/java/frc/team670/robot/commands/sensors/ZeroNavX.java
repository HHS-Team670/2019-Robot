/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.sensors;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;

/**
 * Zeros the NavX yaw at its current position while leaving the field centric angle untouched.
 */
public class ZeroNavX extends InstantCommand {

  public ZeroNavX() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.sensors.zeroYaw();
  }

}
