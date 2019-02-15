/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * Add your docs here.
 */
public class MoveArmAfterDriveDistance extends MoveArm {
  private int inchesToStart;
  private Notifier restrictArmMovement;
  
  /**
   * Add your docs here.
   */
  public MoveArmAfterDriveDistance(ArmState destination, Arm arm, int inchesToStart) {
    super(destination, arm);
    SmartDashboard.putString("current-command", "MoveArmAfterDriveDistance");
    this.inchesToStart = inchesToStart;

    restrictArmMovement = new Notifier(new Runnable() {
      public void run() {
        if (checkDistanceBasedOnLeftEncoder()) {
          initializeSuperclassAndStopNotifier();
        }
      }
    });
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    restrictArmMovement.startPeriodic(0.02);
  }

  private boolean checkDistanceBasedOnLeftEncoder(){
    return (Robot.driveBase.getLeftDIOEncoderPosition() / RobotConstants.DIO_TICKS_PER_INCH < inchesToStart);
  }

  private void initializeSuperclassAndStopNotifier(){
    super.initialize();
    restrictArmMovement.stop();
  }

}
