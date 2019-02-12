/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.elbow;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Controls motors for elbow movement
 */
public class Elbow extends BaseElbow {

  private VictorSPX elbowRotationSlave;

  // Current
  private static final int CURRENT_CONTROL_SLOT = 1;
  private static final int CLIMBING_CONTINUOUS_CURRENT_LIMIT = 35, NORMAL_CONTINUOUS_CURRENT_LIMIT = 33, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps
  private static final double CURRENT_P = 0.2, CURRENT_I = 0.0, CURRENT_D = 0.0, CURRENT_F = 0.0; // TODO Check these constants

  // Motion Magic
  private static final int kPIDLoopIdx = 0, MOTION_MAGIC_SLOT = 0, kTimeoutMs = 0;
  private static final double MM_F = 0, MM_P = 0, MM_I = 0, MM_D = 0; //TODO figure out what these are
  private static final int ELBOW_VELOCITY_SENSOR_UNITS_PER_100_MS = 90; // TODO set this
  private static final int ELBOW_ACCELERATION_SENSOR_UNITS_PER_SEC = 400; // TODO set this
  private static final int OFFSET_FROM_ENCODER_ZERO = 0; // TODO set this
  public static final int FORWARD_SOFT_LIMIT = 850, REVERSE_SOFT_LIMIT = -940; // SET THIS
  private static final int QUAD_ENCODER_MIN = FORWARD_SOFT_LIMIT + 200, QUAD_ENCODER_MAX = REVERSE_SOFT_LIMIT - 200;// SET THIS BASED ON FORWARD AND REVERSE
  private static final double ARBITRARY_FEEDFORWARD = 0; // TODO SET THIS
  private static final double ARBITARY_FEEDFORWARD_FULL_EXTENSION = 0; // Arbitrary feedforward when elbow is fully extended


  public Elbow() {
    super(new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON), ARBITRARY_FEEDFORWARD, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, false, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, NORMAL_CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, OFFSET_FROM_ENCODER_ZERO);
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, rotatorTalon.getDeviceID());  

    rotatorTalon.selectProfileSlot(MOTION_MAGIC_SLOT, kPIDLoopIdx);
		rotatorTalon.config_kF(MOTION_MAGIC_SLOT, MM_F, kTimeoutMs);
		rotatorTalon.config_kP(MOTION_MAGIC_SLOT, MM_P, kTimeoutMs);
		rotatorTalon.config_kI(MOTION_MAGIC_SLOT, MM_I, kTimeoutMs);
    rotatorTalon.config_kD(MOTION_MAGIC_SLOT, MM_D, kTimeoutMs);
    rotatorTalon.configMotionCruiseVelocity(ELBOW_VELOCITY_SENSOR_UNITS_PER_100_MS, kTimeoutMs);
    rotatorTalon.configMotionAcceleration(ELBOW_ACCELERATION_SENSOR_UNITS_PER_SEC, kTimeoutMs);

    /* Config closed loop gains for Primary closed loop (Current) */
    rotatorTalon.config_kP(CURRENT_CONTROL_SLOT, CURRENT_P, RobotConstants.kTimeoutMs);
    rotatorTalon.config_kI(CURRENT_CONTROL_SLOT, CURRENT_I, RobotConstants.kTimeoutMs);
    rotatorTalon.config_kD(CURRENT_CONTROL_SLOT, CURRENT_D, RobotConstants.kTimeoutMs);
    rotatorTalon.config_kF(CURRENT_CONTROL_SLOT, CURRENT_F, RobotConstants.kTimeoutMs);

    rotatorTalon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    rotatorTalon.setNeutralMode(NeutralMode.Brake);
    elbowRotationSlave.setNeutralMode(NeutralMode.Brake);

    stop();
  }

  @Override
  public void setOutput(double output) {
    rotatorTalon.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getOutputCurrent() {
    return rotatorTalon.getOutputCurrent();
  }

  @Override
  public void setClimbingCurrentLimit() {
    rotatorTalon.configContinuousCurrentLimit(CLIMBING_CONTINUOUS_CURRENT_LIMIT);
  }

  @Override
  public void setNormalCurrentLimit() {
    rotatorTalon.configContinuousCurrentLimit(NORMAL_CONTINUOUS_CURRENT_LIMIT);
  }

  @Override
  public double getAngleInDegrees() {
    return convertElbowTicksToDegrees(getPositionTicks());
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new JoystickElbow(this));
  }

  @Override
  public boolean isForwardLimitPressed() {
    //drive until switch is closed
    return rotatorTalon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public boolean isReverseLmitPressed() {
    //drive until switch is closed
    return rotatorTalon.getSensorCollection().isRevLimitSwitchClosed();
  }
  
  @Override
  public void setQuadratureEncoder(double encoderValue) {
    rotatorSensorCollection.setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  @Override
  public void setMotionMagicSetpointAngle(double elbowSetpointAngle) {
    setpoint = convertElbowDegreesToTicks(elbowSetpointAngle);
    rotatorTalon.selectProfileSlot(MOTION_MAGIC_SLOT, kPIDLoopIdx);
    rotatorTalon.set(ControlMode.MotionMagic, setpoint);
  }

  @Override
  public synchronized void setCurrentControl(int current) {
    clearSetpoint();
    rotatorTalon.selectProfileSlot(CURRENT_CONTROL_SLOT, 0);
    rotatorTalon.set(ControlMode.Current, current);
  }

  private static int convertElbowDegreesToTicks(double degrees) {
    // If straight up is 0 and going forward is positive
    // percentage * half rotation
    return (int)((degrees / 360) * TICKS_PER_ROTATION);
  }

  private static double convertElbowTicksToDegrees(double ticks) {
    // If straight up is 0 and going forward is positive
    // percentage * half degrees rotation
    return ((360 * ticks) / TICKS_PER_ROTATION);
  }

  @Override
  protected double getArbitraryFeedForwardAngleMultiplier() {
    double angle = getAngleInDegrees();
    return -1 * Math.sin(Math.toRadians(angle)); // Assumes range of (-180,180) where positive input moves the motor towards 180
  }
}
