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
      DriverStation.reportError("Error instantiating compressor: " + ex.getMessage(), true);
      compressor = null;
    }

    try {
      openClose = new Solenoid(RobotMap.HARD_GRIP_SOLENOID);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating openClose solenoid: " + ex.getMessage(), true);
      openClose = null;
    }

    try {
      hardSoft = new Solenoid(RobotMap.SOFT_GRIP_SOLENOID);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating hardSoft solenoid: " + ex.getMessage(), true);
      hardSoft = null;
    }

    try {
      push = new Solenoid(RobotMap.CLAW_PUSH_SOLENOID);
      push.setPulseDuration(PULSE_DURATION);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating push solenoid: " + ex.getMessage(), true);
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

  /**
   * Closes the claw.
   */
  public void closeClaw(boolean isSoft) {
    updateClaw(true, isSoft);
  }

  /**
   * Opens the claw.
   */
  public void openClaw(boolean isSoft) {
    updateClaw(false, isSoft);
  }

  /**
   * Returns whether the  claw is open or not. If the solenoid isn't connected it will just return false (closed).
   */
  public boolean isOpen() {
    if (openClose != null)
      return openClose.get();
    else
      return false;
  }

  /**
   * Returns whether the claw is set to hard or soft. If solenoid is not connect, will output false (hard).
   * 
   * @return true if soft, false if hard.
   */
  public boolean isSoft() {
    if (openClose != null)
      return hardSoft.get();
    else
      return false;
  }

  /**
   * Should pop out the ball.
   */
  public void push() {
    if (push != null) {
      push.startPulse();
    }
  }

  private void updateClaw(boolean isClosed, boolean isSoft) {
    if (openClose != null)
      openClose.set(isClosed);
    if (hardSoft != null)
      hardSoft.set(isSoft);
  }

  @Override
  public void initDefaultCommand() {

  }
}
