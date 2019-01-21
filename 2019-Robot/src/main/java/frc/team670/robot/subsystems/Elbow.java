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
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Controls motors for elbow movement
 */
public class Elbow extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX elbowRotationMain;
  private VictorSPX elbowRotationSlave;
  private double elbowAngle;
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  public Elbow() {  
    elbowRotationMain = new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON);   
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());  
    elbowRotationMain.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		elbowRotationMain.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		elbowRotationMain.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		elbowRotationMain.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    elbowRotationMain.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
  }

/**
   * Sets the peak current limit for the elbow motor.
   * @param current Current in amps
   */
  public void setCurrentLimit(int current) {
    elbowRotationMain.configPeakCurrentLimit(RobotConstants.PEAK_AMPS, RobotConstants.TIMEOUT_MS); // Peak Limit at 0
    elbowRotationMain.configPeakCurrentDuration(RobotConstants.PEAK_TIME_MS, RobotConstants.TIMEOUT_MS); // Duration at over peak set to 0
    elbowRotationMain.configContinuousCurrentLimit(current, RobotConstants.TIMEOUT_MS);
  }

  public void enableCurrentLimit() {
    elbowRotationMain.enableCurrentLimit(true);
  }

  public void disableCurrentLimit() {
    elbowRotationMain.enableCurrentLimit(false);
  }

  public void setOutput(double output){
    elbowRotationMain.set(ControlMode.PercentOutput, output);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public TalonSRX getTalon() {
    return this.elbowRotationMain;
  }

  /**
   * Setup for movement and Motion Magic
   */
  public void moveElbow(double elbowAngle) {
		elbowRotationMain.configMotionCruiseVelocity(15000, kTimeoutMs);
		elbowRotationMain.configMotionAcceleration(6000, kTimeoutMs);
    elbowRotationMain.set(ControlMode.MotionMagic, elbowAngle);
  }

}
