/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.extension.Extension;

/**
 * Add your docs here.
 */
public class JoystickExtension extends InstantCommand {

  private BaseExtension extension;
  private BaseIntake intake;
  private Arm arm;
  private double power;
  public static final double OPERATOR_EXTENSION_CONTROL_SCALAR = 0.5;
  private static final int INCH_TOLERANCE = 1;

  public JoystickExtension(Arm arm, BaseIntake intake, BaseExtension extension, double power) {
    super();
    this.extension = extension;
    this.power = power;
    this.arm = arm;
    requires(extension);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    power *= (power > 0) ? OPERATOR_EXTENSION_CONTROL_SCALAR * Extension.MAX_EXTENSION_OUTPUT : -1 * OPERATOR_EXTENSION_CONTROL_SCALAR * Extension.MAX_EXTENSION_OUTPUT ;
  

    // Front pistons are approaching flat position so either the entire robot is
    // almost back down or the front pistons are almost retracted. If the original
    // input is already at -0.05 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (power < -0.05 && extension.getLengthInches() <= extension.getReverseSoftLimitInInches() + INCH_TOLERANCE) {
      power = Math.min(-0.05, (power * (Math.abs(extension.getLengthInches() - (extension.getReverseSoftLimitInInches() + INCH_TOLERANCE)) / INCH_TOLERANCE)));
    }
    // Front pistons are approaching fully deployed position. If the original input
    // is already at 0.1 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (power > 0.1 && extension.getLengthInches() >= extension.getForwardSoftLimitInInches() - INCH_TOLERANCE) {
      power *= Math.max(0.1, (power * (extension.getForwardSoftLimitInInches() - INCH_TOLERANCE) / INCH_TOLERANCE));
    }

    //ADD MORE CHECKS FOR IF ARM GOES THROUGH INTAKE
    if (Arm.getCoordPosition(arm.getElbow().getAngleInDegrees(), arm.getWrist().getAngleInDegrees(), extension.getLengthInches()).getX() >= intake.getIntakeCoordinates().getX() - INCH_TOLERANCE) {
      if (Arm.getCoordPosition(arm.getElbow().getAngleInDegrees(), arm.getWrist().getAngleInDegrees(), extension.getLengthInches()).getY() <= intake.getIntakeCoordinates().getY() + INCH_TOLERANCE) {
        power = 0;
      }
    }

    extension.setOutput(power);
  }
}
