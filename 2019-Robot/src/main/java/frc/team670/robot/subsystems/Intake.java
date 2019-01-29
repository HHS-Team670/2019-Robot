/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Represents the Intake mechanism of the robot.
 */
public class Intake extends Subsystem {

  public static final int INTAKE_ANGLE_UP = 0, INTAKE_ANGLE_DOWN = 0; //TODO set these

  private VictorSPX baseVictor, rollerVictor;
  private Encoder baseVictorEncoder;

  private static final double MIN_BASE_OUTPUT = -0.75, MAX_BASE_OUTPUT = 0.75;


  private static final double kF = 0, kP = 0.1, kI = 0, kD = 0; //TODO figure out what these are
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0; //TODO Set this
  private final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in rotations
  private static final double RAMP_RATE = 0.1;


  public Intake() {
    baseVictor = new VictorSPX(RobotMap.INTAKE_BASE_VICTOR);
    rollerVictor = new VictorSPX(RobotMap.INTAKE_ROLLER_VICTOR);

    baseVictorEncoder = new Encoder(RobotMap.INTAKE_BASE_ENCODER_CHANNEL_A, RobotMap.INTAKE_BASE_ENCODER_CHANNEL_B, false, EncodingType.k4X);

    enableBaseMotionMagic();
  }

  // May need to set tolerance
  private void enableBaseMotionMagic(){
    baseVictor.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		baseVictor.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		baseVictor.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		baseVictor.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    baseVictor.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    baseVictor.configMotionCruiseVelocity(RobotConstants.MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    baseVictor.configMotionAcceleration(RobotConstants.MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    

    // These thresholds stop the motor when limit is reached
    baseVictor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    baseVictor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

    // Enable Safety Measures
    baseVictor.configForwardSoftLimitEnable(true);
    baseVictor.configReverseSoftLimitEnable(true);

    baseVictor.setNeutralMode(NeutralMode.Brake);
    rollerVictor.setNeutralMode(NeutralMode.Coast);

    baseVictor.configClosedloopRamp(RAMP_RATE);
    baseVictor.configOpenloopRamp(RAMP_RATE);
  }

  /**
   * Should set the setpoint for the Motion Magic on the intake
   */
  public void setMotionMagicSetpoint(double intakeAngle) {  
    baseVictor.set(ControlMode.MotionMagic, MathUtils.convertIntakeDegreesToTicks(intakeAngle));
  }

  /**
   * Should return the setpoint for the motion magic on the base motor
   */
  public double getMotionMagicSetpoint(){
    return baseVictor.getClosedLoopTarget(); 
  }

  public void setRotatorNeutralMode(NeutralMode mode) {
    baseVictor.setNeutralMode(mode);
  }

  /**
   * Returns the tick value of the base motor
   */
  public int getIntakePositionInTicks(){
    return baseVictorEncoder.get();
  }

  /**
   * Returns the intake angle in degrees
   */
  public double getIntakeAngleInDegrees(){
    return MathUtils.convertIntakeTicksToDegrees(baseVictorEncoder.get());
  }


  /**
   * Runs the intake at a given percent power
   * 
   * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
   */
  public void runIntake(double power) {
    rollerVictor.set(ControlMode.PercentOutput, power);
  }


  @Override
  public void initDefaultCommand() {

  }
}
