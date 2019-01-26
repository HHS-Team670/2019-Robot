/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.ArmControlMode;
import frc.team670.robot.subsystems.extension.Extension;

/**
 * Add your docs here.
 */
public class JoystickExtension extends InstantCommand {
  
  private Extension extension;

  public JoystickExtension(Extension extension) {
    super();
    this.extension = extension;
    requires(extension);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (FlipJoystickArmControl.state.equals(ArmControlMode.EXTENSION)) {
      extension.setOutput(RobotConstants.OPERATOR_ARM_CONTROL_SCALAR * Robot.oi.getOperatorController().getRightStickY());
    }
  }
}