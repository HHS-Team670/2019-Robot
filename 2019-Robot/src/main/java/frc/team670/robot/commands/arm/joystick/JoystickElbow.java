/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.utils.ArmControlMode;

/**
 * Add your docs here.
 */
public class JoystickElbow extends InstantCommand {
  
  private BaseElbow elbow;
  private double power;

  public JoystickElbow(BaseElbow elbow, double power) {
    super();
    this.elbow = elbow;
    this.power = power;
    requires(elbow);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (FlipJoystickArmControl.state.equals(ArmControlMode.ELBOW)) {
      elbow.setOutput(Arm.OPERATOR_ARM_CONTROL_SCALAR * power);
    }
  }
}
