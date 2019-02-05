/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ResetEncoder extends InstantCommand {
  private RotatingSubsystem rotatingSubsystem;

  public ResetEncoder(RotatingSubsystem rotatingSubsystem) {
    this.rotatingSubsystem = rotatingSubsystem;
    requires(rotatingSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rotatingSubsystem.rotatorTalon.getSensorCollection().setQuadraturePosition(0, 10);
  }


}
