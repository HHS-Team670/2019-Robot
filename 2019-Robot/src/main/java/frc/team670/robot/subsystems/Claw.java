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

  /** The amount of time to delay to allow the pneumatics to move in seconds */
  public static final double TIME_TO_MOVE = 0.35;

  private static final double PULSE_DURATION = 0.4; // In seconds
  public static final double MAX_CLAW_OPEN_DIAMETER = 20; //Set distance
  // Used to find lowest point on arm (so that intake doesn't crash into it). If claw is angled up, the extension tip is lowest point.

  /**
   * Distance from base of claw to the end if it is closed.
   */
  public static final double LENGTH_IN_INCHES = 12;

  private Compressor compressor;
  private Solenoid sol0, sol1, push;

  /*
   * Hold hatch plate when disabled, so false = open, true = closed
   * 
   * Solenoid 0 when off puts air pushing out
   * Soleonid 1 when on puts air pushing in
   * 
   */
  public Claw() {
    try {
      compressor = new Compressor(RobotMap.PC_MODULE);
      compressor.setClosedLoopControl(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating compressor: " + ex.getMessage(), true);
      compressor = null;
    }

    try {
      sol0 = new Solenoid(RobotMap.SOLENOID_0);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating openClose solenoid: " + ex.getMessage(), true);
      sol0 = null;
    }

    try {
      sol1 = new Solenoid(RobotMap.SOLENOID_1);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating hardSoft solenoid: " + ex.getMessage(), true);
      sol1 = null;
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
    if (sol1 != null) {
      changeSolenoid(!sol1.get());
    }
  }

  /**
   * Closes the claw.
   */
  public void closeClaw(boolean isSoft) {
    changeSolenoid(false);
  }

  /**
   * Opens the claw.
   */
  public void openClaw() {
    changeSolenoid(true);
  }


  /**
   * @param open true to open, false to close
   */
  private void changeSolenoid(boolean open) {
    if(sol1 != null && sol0 != null) {
      sol0.set(!open);
      sol1.set(open);
    }
  }

  /**
   * Should pop out the ball.
   */
  public void push() {
    if (push != null) {
      push.startPulse();
    }
  }

  public boolean isOpen() {
    if(sol1 != null) {
      return sol1.get();
    }
    else if (sol0 != null) {
      return !sol0.get();
    }
    return true;
  }

  @Override
  public void initDefaultCommand() {

  }
}
