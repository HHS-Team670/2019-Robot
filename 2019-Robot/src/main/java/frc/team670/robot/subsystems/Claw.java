/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents the claw mechanism of the robot.
 * Release Ball: hard open, push
 * Pick Up Ball: soft close
 * Pick Up Hatch: hard open
 * Release Hatch: hard close
 * Opening Normally: soft open
 * @author shaylandias
 */
public class Claw extends Subsystem {

  private static double PULSE_DURATION = 0.4; // In seconds

  private Compressor compressor;
  private Solenoid openClose, hardSoft, push;

  public Claw() {
    compressor = new Compressor(RobotMap.PC_MODULE);
    compressor.setClosedLoopControl(true);

    openClose = new Solenoid(RobotMap.HARD_GRIP_SOLENOID);
    hardSoft = new Solenoid(RobotMap.SOFT_GRIP_SOLENOID);
    push = new Solenoid(RobotMap.CLAW_PUSH_SOLENOID);
    push.setPulseDuration(PULSE_DURATION);
  }

  /**
   * Toggles the claw grip based on closed and soft.
   */
  public void toggleGrip() {
    updateClaw(!openClose.get(), true);
  }

  public void closeClaw(boolean isSoft) {
    updateClaw(true, isSoft);
  }

  public void openClaw(boolean isSoft) {
    updateClaw(false, isSoft);
  }

  private void updateClaw(boolean isClosed, boolean isSoft) {
    openClose.set(isClosed);
    hardSoft.set(isSoft);
  }

  public void push() {
    push.startPulse();
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
