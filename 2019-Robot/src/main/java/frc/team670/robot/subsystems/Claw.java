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
 * @author shaylandias
 */
public class Claw extends Subsystem {

  private static double PULSE_DURATION = 0.4; // In seconds

  private Compressor compressor;
  private Solenoid hard, soft, push;
  private boolean open;

  public Claw() {

    compressor = new Compressor(RobotMap.PC_MODULE);
    compressor.setClosedLoopControl(true);

    open = false;
    hard = new Solenoid(RobotMap.HARD_GRIP_SOLENOID);
    soft = new Solenoid(RobotMap.SOFT_GRIP_SOLENOID);
    push = new Solenoid(RobotMap.CLAW_PUSH_SOLENOID);
    push.setPulseDuration(PULSE_DURATION);
  }

  /**
   * Toggles the claw grip based on closed and soft
   * @param closed True if claw should be closed, false if claw should be open
   * @param soft True if claw should use soft grip, false if claw should use hard grip. If closed is false, this will not affect anything.
   */
  public void toggleGrip(boolean closed, boolean soft) {
    if(soft) {
      hard.set(false);
      this.soft.set(open);
    }
    else {
      this.soft.set(open);
      this.hard.set(open);
    }

  }

  public void push() {
    push.startPulse();
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
