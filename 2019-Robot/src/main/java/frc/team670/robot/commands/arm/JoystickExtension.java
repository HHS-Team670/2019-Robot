/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.ArmControlMode;

/**
 * Add your docs here.
 */
public class JoystickExtension extends InstantCommand {
  /**
   * Add your docs here.
   */

  public JoystickExtension() {
    super();
    requires(Robot.extension);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (FlipJoystickArmControl.state == ArmControlMode.EXTENSION) {
      if (Robot.oi.getOperatorController().getRightStickY() < 0.5) {
        Robot.extension.setOutput(Robot.oi.getOperatorController().getRightStickY());
      }
    }
  }
}
