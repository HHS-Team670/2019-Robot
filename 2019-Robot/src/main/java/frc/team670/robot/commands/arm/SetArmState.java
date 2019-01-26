/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * Sets teh current ArmState to the provided ArmState state
 */
public class SetArmState extends InstantCommand {
  
  private ArmState state;

  public SetArmState(ArmState state) {
    super();
    this.state = state;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Arm.setState(state);
  }

}
