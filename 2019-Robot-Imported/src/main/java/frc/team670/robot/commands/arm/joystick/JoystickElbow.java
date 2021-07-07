/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.elbow.Elbow;

/**
 * Add your docs here.
 */
public class JoystickElbow extends InstantCommand {

  private BaseElbow elbow;
  private double power;
  private static final double DEGREE_TOLERANCE = 1;
  private static final double OPERATOR_ELBOW_CONTROL_SCALAR = 0.5;

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
    power *= (power > 0) ? OPERATOR_ELBOW_CONTROL_SCALAR * Elbow.MAX_ELBOW_OUTPUT : -1 * OPERATOR_ELBOW_CONTROL_SCALAR * Elbow.MAX_ELBOW_OUTPUT ;
  

   // Front pistons are approaching flat position so either the entire robot is
    // almost back down or the front pistons are almost retracted. If the original
    // input is already at -0.05 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (power < -0.05 && elbow.getAngleInDegrees() <= elbow.getReverseSoftLimitAngle() + DEGREE_TOLERANCE) {
      power = Math.min(-0.05, (power * (Math.abs(elbow.getAngleInDegrees() - (elbow.getReverseSoftLimitAngle() + DEGREE_TOLERANCE)) / DEGREE_TOLERANCE)));
    }
    // Front pistons are approaching fully deployed position. If the original input
    // is already at 0.1 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (power > 0.1 && elbow.getAngleInDegrees() >= elbow.getForwardSoftLimitAngle()- DEGREE_TOLERANCE) {
      power *= Math.max(0.1, (power * (elbow.getForwardSoftLimitAngle() - elbow.getAngleInDegrees()) / DEGREE_TOLERANCE));
    }
  }
}
