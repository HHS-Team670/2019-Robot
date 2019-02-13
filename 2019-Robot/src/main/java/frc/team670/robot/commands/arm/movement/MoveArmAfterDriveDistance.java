/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * Add your docs here.
 */
public class MoveArmAfterDriveDistance extends MoveArm {
  private int inchesToStart;
  /**
   * Add your docs here.
   */
  public MoveArmAfterDriveDistance(ArmState destination, Arm arm, int inchesToStart) {
    super(destination, arm);
    this.inchesToStart = inchesToStart;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    while (Robot.driveBase.getLeftDIOEncoderPosition() / RobotConstants.DIO_TICKS_PER_INCH < inchesToStart) {
      // do nothing
    }
    // do this after the loop exits
    super.initialize();
  }

}
