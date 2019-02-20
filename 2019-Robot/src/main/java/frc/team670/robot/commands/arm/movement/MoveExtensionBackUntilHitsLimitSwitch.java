/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.extension.Extension;

/**
 * Add your docs here.
 */
public class MoveExtensionBackUntilHitsLimitSwitch extends Command {

  private BaseExtension extension;

  public MoveExtensionBackUntilHitsLimitSwitch(BaseExtension extension) {
    super(extension);
    this.extension = extension;
    setInterruptible(false);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  
  }

  protected void execute() {
    double value = extension.getArbitraryFeedForwardAngleMultiplier() * Extension.ARBITRARY_FEEDFORWARD_CONSTANT;
    extension.moveByPercentOutput(-0.03 + value);
  }

  @Override
  protected boolean isFinished() {
    return extension.getReverseLimitSwitchTripped();
  }

  protected void end(){
    extension.stop();
    extension.resetLimitSwitch();
  }

  @Override
  protected void interrupted() {
   
  }

}
