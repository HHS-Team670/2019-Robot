/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.constants.RobotConstants;

/**
 * Controls motors for elbow movement
 */
public class Elbow extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX elbowRotationMain;
  private VictorSPX elbowRotationSlave;

  private static final int CURRENT_CONTROL_SLOT = 0; // TODO Set this
  private final int CLIMBING_CONTINUOUS_CURRENT_LIMIT = 35, NORMAL_CONTINUOUS_CURRENT_LIMIT = 33, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps

  private double currentP = 0.2, currentI = 0.0, currentD = 0.0, currentF = 0.0; // TODO Check these constants

  public Elbow() {
    elbowRotationMain = new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON);
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());
  }

  /**
   * Sets the output for the elbow motor
   * 
   * @param output the desired output
   */
  public void setOutput(double output) {
    elbowRotationMain.set(ControlMode.PercentOutput, output);
  }

  /**
   * Returns the output current
   * 
   * @return the output current of the elbow motor
   */
  public double getOutputCurrent() {
    return elbowRotationMain.getOutputCurrent();
  }

  /**
   * Sets the current limit for when the robot begins to climb
   */
  public void setClimbingCurrentLimit() {
    elbowRotationMain.configContinuousCurrentLimit(CLIMBING_CONTINUOUS_CURRENT_LIMIT);
  }

  /**
   * Resets the currnet limit to its normal value
   */
  public void setNormalCurrentLimit() {
    elbowRotationMain.configContinuousCurrentLimit(NORMAL_CONTINUOUS_CURRENT_LIMIT);
  }

  /**
   * Returns the angle of the elbow with the arm as a zero
   * 
   * @return the angle of the elbow with the arm as a zero
   */
  public double getElbowAngle() {
    return 0.0; // TODO convert the actual tick value to an angle
  }

  /**
   * Returns the main talon to control the elbow
   * 
   * @return the main talon to control the elbow
   */
  public TalonSRX getElbowTalon() {
    return elbowRotationMain;
  }

  /**
   * Should create a closed loop for the current to hold the elbow down
   */
  public void setCurrentClosedLoopToHoldElbowDown() {
    /* Factory default hardware to prevent unexpected behaviour */
    elbowRotationMain.configFactoryDefault();

    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    elbowRotationMain.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    elbowRotationMain.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    elbowRotationMain.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    elbowRotationMain.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table here for units to use:
     * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
     */
    elbowRotationMain.configAllowableClosedloopError(0,CURRENT_CONTROL_SLOT, RobotConstants.kTimeoutMs);

    /* Config closed loop gains for Primary closed loop (Current) */
    elbowRotationMain.config_kP(CURRENT_CONTROL_SLOT, currentP, RobotConstants.kTimeoutMs);
    elbowRotationMain.config_kI(CURRENT_CONTROL_SLOT, currentI, RobotConstants.kTimeoutMs);
    elbowRotationMain.config_kD(CURRENT_CONTROL_SLOT, currentD, RobotConstants.kTimeoutMs);
    elbowRotationMain.config_kF(CURRENT_CONTROL_SLOT, currentF, RobotConstants.kTimeoutMs);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
