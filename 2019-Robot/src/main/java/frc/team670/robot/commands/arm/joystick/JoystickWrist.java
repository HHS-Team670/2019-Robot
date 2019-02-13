/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.subsystems.wrist.Wrist;

/**
 * Add your docs here.
 */
public class JoystickWrist extends InstantCommand {

  private BaseWrist wrist;
  private double power;
  private static final double OPERATOR_WRIST_CONTROL_SCALAR = 0.5;
  private static final double DEGREE_TOLERANCE = 3;

  public JoystickWrist(BaseWrist wrist, double power) {
    super();
    this.wrist = wrist;
    this.power = power;
    requires(wrist);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    power *= (power > 0) ? OPERATOR_WRIST_CONTROL_SCALAR * Wrist.MAX_WRIST_OUTPUT : -1 * OPERATOR_WRIST_CONTROL_SCALAR *  Wrist.MAX_WRIST_OUTPUT; //Multiplied by -1 because constant is negative and so is stick input
  

    // Front pistons are approaching flat position so either the entire robot is
    // almost back down or the front pistons are almost retracted. If the original
    // input is already at -0.05 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (power < -0.05 && wrist.getAngleInDegrees() <= wrist.getReverseSoftLimitAngle() + DEGREE_TOLERANCE) {
      power = Math.min(-0.05, (power * (Math.abs(wrist.getAngleInDegrees() - (wrist.getReverseSoftLimitAngle() + DEGREE_TOLERANCE)) / DEGREE_TOLERANCE)));
    }
    // Front pistons are approaching fully deployed position. If the original input
    // is already at 0.1 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (power > 0.1 && wrist.getAngleInDegrees() >= wrist.getForwardSoftLimitAngle()  - DEGREE_TOLERANCE) {
      power *= Math.max(0.1, (power * ( wrist.getForwardSoftLimitAngle() - wrist.getAngleInDegrees()) / DEGREE_TOLERANCE));
    }

    wrist.setOutput(power);

  }
}
