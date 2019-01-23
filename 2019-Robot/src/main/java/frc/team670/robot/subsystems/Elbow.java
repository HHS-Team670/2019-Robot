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
import frc.team670.robot.utils.functions.MathUtils;

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

  private final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in encoder rotations
  private final int CURRENT_LIMIT = 0; // TODO set current limit in Amps

  public Elbow() {
    elbowRotationMain = new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON);
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());  
    elbowRotationMain.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		elbowRotationMain.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		elbowRotationMain.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		elbowRotationMain.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    elbowRotationMain.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    elbowRotationMain.configMotionCruiseVelocity(RobotConstants.MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    elbowRotationMain.configMotionAcceleration(RobotConstants.MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);

    // These thresholds stop the motor when limit is reached
    elbowRotationMain.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    elbowRotationMain.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
    elbowRotationMain.configContinuousCurrentLimit(CURRENT_LIMIT);

    elbowRotationMain.configForwardSoftLimitEnable(true);
    elbowRotationMain.configReverseSoftLimitEnable(true);
    elbowRotationMain.enableCurrentLimit(true);
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
   * Selects the PID Slot dedicated to MotionMagic to give it the correct PID Values
   */
  public void initializeMotionmagic() {
    elbowRotationMain.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
  }

  public int getPositionTicks() {
    return elbowRotationMain.getSensorCollection().getQuadraturePosition();
  }
  
  public double getAngle() {
    return MathUtils.convertElbowTicksToDegrees(getPositionTicks());
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

  public TalonSRX getTalon() {
    return this.elbowRotationMain;
  }

  /**
   * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
   */
  public void resetElbow(double encoderValue) {
    elbowRotationMain.getSensorCollection().setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  /**
   * Setup for movement and Motion Magic
   */
  public void setMotionMagicSetpoint(double elbowAngle) {
    elbowRotationMain.set(ControlMode.MotionMagic, MathUtils.convertElbowDegreesToTicks(elbowAngle));
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
