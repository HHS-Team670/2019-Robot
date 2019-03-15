/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.zero;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveElbow;
import frc.team670.robot.commands.arm.movement.MoveExtensionBackUntilHitsLimitSwitch;
import frc.team670.robot.Robot;

public class SafelyResetExtension extends CommandGroup {

  public SafelyResetExtension() {
    requires(Robot.arm.getElbow());
    addSequential(new MoveElbow(Robot.arm.getElbow(), 50.0));
    addSequential(new MoveExtensionBackUntilHitsLimitSwitch(Robot.arm.getExtension()));

  }
}
