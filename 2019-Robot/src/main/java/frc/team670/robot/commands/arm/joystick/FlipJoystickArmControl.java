/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.joystick;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.utils.ArmControlMode;

/**
 * Add your docs here.
 */
public class FlipJoystickArmControl extends InstantCommand {
  /**
   * Add your docs here.
   */
  public static ArmControlMode state;

  public FlipJoystickArmControl() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    state = ArmControlMode.DISABLED;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (state.equals(ArmControlMode.DISABLED)) {
      state = ArmControlMode.ELBOW;
    } else if (state.equals(ArmControlMode.ELBOW)) {
      state = ArmControlMode.EXTENSION;
    } else if (state.equals(ArmControlMode.EXTENSION)) {
      state = ArmControlMode.WRIST;
    } else {
      state = ArmControlMode.DISABLED;
    }
  }

}
