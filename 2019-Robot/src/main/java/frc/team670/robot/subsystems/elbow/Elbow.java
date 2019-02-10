/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.elbow;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.team670.robot.commands.arm.joystick.JoystickElbow;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Controls motors for elbow movement
 */
public class Elbow extends BaseElbow {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private VictorSPX elbowRotationSlave;
  public static double ELBOW_START_POS = 6000; // TODO Set this
  public static final double MAX_ELBOW_BACK = 0; //TODO find what is this number
  public static final double MAX_ELBOW_FORWARD = 0; //TODO also find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  private static final int OFFSET_FROM_ENCODER_ZERO = 0;

  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  private static final int CURRENT_CONTROL_SLOT = 0; // TODO Set this
  private final int CLIMBING_CONTINUOUS_CURRENT_LIMIT = 35, NORMAL_CONTINUOUS_CURRENT_LIMIT = 33, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps

  private double currentP = 0.2, currentI = 0.0, currentD = 0.0, currentF = 0.0; // TODO Check these constants

  private static int ELBOW_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
  private static int ELBOW_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this

  public static final int QUAD_ENCODER_MAX = 890, QUAD_ENCODER_MIN = -1158; //TODO Set these values

  public Elbow() {
    super(new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON),  0.0, 0, 0, false, 0, 0, 33, 0, OFFSET_FROM_ENCODER_ZERO);
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, rotatorTalon.getDeviceID());  

    rotatorTalon.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		rotatorTalon.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		rotatorTalon.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		rotatorTalon.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    rotatorTalon.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    rotatorTalon.configMotionCruiseVelocity(ELBOW_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    rotatorTalon.configMotionAcceleration(ELBOW_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);

    /* Config closed loop gains for Primary closed loop (Current) */
    rotatorTalon.config_kP(CURRENT_CONTROL_SLOT, currentP, RobotConstants.kTimeoutMs);
    rotatorTalon.config_kI(CURRENT_CONTROL_SLOT, currentI, RobotConstants.kTimeoutMs);
    rotatorTalon.config_kD(CURRENT_CONTROL_SLOT, currentD, RobotConstants.kTimeoutMs);
    rotatorTalon.config_kF(CURRENT_CONTROL_SLOT, currentF, RobotConstants.kTimeoutMs);

    rotatorTalon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    rotatorTalon.setNeutralMode(NeutralMode.Brake);
    elbowRotationSlave.setNeutralMode(NeutralMode.Brake);

    enablePercentOutput();
 
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
    rotatorTalon.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
    rotatorTalon.set(ControlMode.MotionMagic, setpoint);
  }

  @Override
  public void setCurrentControl(int current) {
    rotatorTalon.selectProfileSlot(CURRENT_CONTROL_SLOT, 0);
    rotatorTalon.set(ControlMode.Current, current);
  }

  private static int convertElbowDegreesToTicks(double degrees) {
    // If straight up is 0 and going forward is positive
    // percentage * half rotation
    return (int) ((degrees / 180) * (0.5 * RobotConstants.ELBOW_TICKS_PER_ROTATION));
  }

  private static double convertElbowTicksToDegrees(double ticks) {
    // If straight up is 0 and going forward is positive
    // percentage * half degrees rotation
    return (ticks / (0.5 * RobotConstants.ELBOW_TICKS_PER_ROTATION)) * 180;
  }

  @Override
  protected double getArbitraryFeedForwardAngleMultiplier() {
    return (-1.0 * Math.cos(Math.toRadians(getAngleInDegrees())));
  }
}
