/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.Claw;

/**
 * Add your docs here.
 */
public class ToggleClaw extends InstantCommand {
  
  private Claw claw;

  public ToggleClaw(Claw claw) {
    super();
    requires(claw);
    this.claw = claw;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "ToggleClaw");
    claw.toggleGrip();
    Robot.arm.setHeldItem(HeldItem.NONE);
  }

}
