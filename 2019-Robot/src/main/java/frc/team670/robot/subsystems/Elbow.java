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

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
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

  public Elbow() {
    elbowRotationMain = new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON);
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());
  }

  /**
   * Sets the peak current limit for the elbow motor.
   * 
   * @param current Current in amps
   */
  public void setCurrentLimit(int current) {
    elbowRotationMain.configPeakCurrentLimit(RobotConstants.PEAK_AMPS, RobotConstants.TIMEOUT_MS); // Peak Limit at 0
    elbowRotationMain.configPeakCurrentDuration(RobotConstants.PEAK_TIME_MS, RobotConstants.TIMEOUT_MS); // Duration at
                                                                                                         // over peak
                                                                                                         // set to 0
    elbowRotationMain.configContinuousCurrentLimit(current, RobotConstants.TIMEOUT_MS);
  }

  /**
   * Enables the current limit for the elbow motor
   */
  public void enableCurrentLimit() {
    elbowRotationMain.enableCurrentLimit(true);
  }


  /**
   * Disables the current limit for the elbow motor
   */
  public void disableCurrentLimit() {
    elbowRotationMain.enableCurrentLimit(false);
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

  public double getElbowAngle() {
    return 0.0; // TODO convert the actual tick value to an angle
  }

  public TalonSRX getElbowTalon(){
    return elbowRotationMain;
  }

  public ElbowAngle_PIDSource getElbowAngle_PIDSource(){
    return new ElbowAngle_PIDSource();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
public class ElbowAngle_PIDSource implements PIDSource {
  private PIDSourceType type;

  public ElbowAngle_PIDSource(){
    type = PIDSourceType.kDisplacement;
  } 

  @Override
  public PIDSourceType getPIDSourceType() {
    return type;
  }

  @Override
  public double pidGet() {
    return getElbowAngle();
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
    this.type = pidSource;
  }
  }
}
