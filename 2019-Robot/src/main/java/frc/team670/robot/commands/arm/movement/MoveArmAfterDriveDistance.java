/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * Add your docs here.
 */
public class MoveArmAfterDriveDistance extends InstantCommand {
  private int inchesToStart;
  private Notifier restrictArmMovement;
  private double initialLeftEncoderPosition;
  
  /**
   * @param initialLeftEncoderPosition Left Encoder position in inches
   */
  public MoveArmAfterDriveDistance(ArmState destination, Arm arm, int inchesToStart, double initialLeftEncoderPosition) {
    SmartDashboard.putString("current-command", "MoveArmAfterDriveDistance");
    this.inchesToStart = inchesToStart;
    this.initialLeftEncoderPosition = initialLeftEncoderPosition;

    restrictArmMovement = new Notifier(new Runnable() {
      public void run() {
        if (hasDrivenDistance()) {
          Scheduler.getInstance().add(new MoveArm(destination, arm));
          restrictArmMovement.stop();
        }
      }
    });
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    restrictArmMovement.startPeriodic(0.02);
  }

  private boolean hasDrivenDistance(){
    return (Math.abs(Robot.driveBase.getLeftMustangEncoderPositionInInches() - initialLeftEncoderPosition) > inchesToStart);
  }

}
