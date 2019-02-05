/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.utils.ArmControlMode;

/**
 * Add your docs here.
 */
public class JoystickWrist extends InstantCommand {
  
  private Wrist wrist;

  public JoystickWrist(Wrist wrist) {
    super();
    this.wrist = wrist;
    requires(wrist);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (FlipJoystickArmControl.state.equals(ArmControlMode.WRIST)) {
      wrist.setOutput(Arm.OPERATOR_ARM_CONTROL_SCALAR * Robot.oi.getOperatorController().getRightStickY());
    }
  }
}