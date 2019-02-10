/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.OI;

/**
 * Sets the rumble on the driver and operator controllers
 */
public class RumbleOperatorController extends InstantCommand {
  
  private OI oi;
  private double power, time;

    /**
     * @param power The desired power of the rumble [0, 1]
     * @param time The time to rumble for in seconds
     */
  public RumbleOperatorController(OI oi, double power, double time) {
    super();
    this.oi = oi;
    this.power = power;
    this.time = time;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    oi.rumbleOperatorController(power, time);
  }

}
