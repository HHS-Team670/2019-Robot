/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.subsystems.TunableSubsystem;

public class ResetPulseWidthEncoder extends InstantCommand {
  private TunableSubsystem tunableSubsystem;

  public ResetPulseWidthEncoder(TunableSubsystem tunableSubsystem) {
    this.tunableSubsystem = tunableSubsystem;
    requires((Subsystem)tunableSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // tunableSubsystem.zeroPulseWidthEncoder();
  }


}
