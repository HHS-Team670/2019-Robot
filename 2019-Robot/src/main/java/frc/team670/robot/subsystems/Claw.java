/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the claw mechanism of the robot. Release Ball: hard open, push
 * Pick Up Ball: soft close Pick Up Hatch: hard open Release Hatch: hard close
 * Opening Normally: soft open
 * 
 * @author shaylandias
 */
public class Claw extends Subsystem {

  private static double PULSE_DURATION = 0.4; // In seconds

  private Compressor compressor;
  private Solenoid openClose, hardSoft, push;

  public Claw() {
    try {
      compressor = new Compressor(RobotMap.PC_MODULE);
      compressor.setClosedLoopControl(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating compressor : " + ex.getMessage(), true);
      compressor = null;
    }

    try {
      openClose = new Solenoid(RobotMap.HARD_GRIP_SOLENOID);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating openClose solenoid :" + ex.getMessage(), true);
      openClose = null;
    }

    try {
      hardSoft = new Solenoid(RobotMap.SOFT_GRIP_SOLENOID);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating hardSoft solenoid :" + ex.getMessage(), true);
      hardSoft = null;
    }

    try {
      push = new Solenoid(RobotMap.CLAW_PUSH_SOLENOID);
      push.setPulseDuration(PULSE_DURATION);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating push solenoid :" + ex.getMessage(), true);
      push = null;
    }

  }

  /**
   * Toggles the claw grip based on closed and soft.
   */
  public void toggleGrip() {
    if (openClose != null)
      updateClaw(!openClose.get(), true);
  }

  public void closeClaw(boolean isSoft) {
    updateClaw(true, isSoft);
  }

  public void openClaw(boolean isSoft) {
    updateClaw(false, isSoft);
  }

  public boolean isOpen() {
    return openClose.get();
  }

  /**
   * @return true if soft, false if hard.
   */
  public boolean isSoft() {
    return hardSoft.get();
  }

  private void updateClaw(boolean isClosed, boolean isSoft) {
    if (openClose != null && hardSoft != null) {
      openClose.set(isClosed);
      hardSoft.set(isSoft);
    }
  }

  public void push() {
    if (push != null) {
      push.startPulse();
    }
  }

  @Override
  public void initDefaultCommand() {

  }
}
