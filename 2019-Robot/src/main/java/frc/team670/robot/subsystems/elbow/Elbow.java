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


  private TalonSRX elbowRotationMain;
  private VictorSPX elbowRotationSlave;
  public static double ELBOW_START_POS = 6000; // TODO Set this
  public static final double MAX_ELBOW_BACK = 0; //TODO find what is this number
  public static final double MAX_ELBOW_FORWARD = 0; //TODO also find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  private static final double RAMP_RATE = 0.1;

  private final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in encoder rotations
  private static final int CURRENT_CONTROL_SLOT = 0; // TODO Set this
  private final int CLIMBING_CONTINUOUS_CURRENT_LIMIT = 35, NORMAL_CONTINUOUS_CURRENT_LIMIT = 33, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps

  private double currentP = 0.2, currentI = 0.0, currentD = 0.0, currentF = 0.0; // TODO Check these constants

  private static int ELBOW_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
  private static int ELBOW_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this

  public Elbow() {
    elbowRotationMain = new TalonSRX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_TALON);
    super.setTalon(elbowRotationMain);
    elbowRotationSlave = new VictorSPX(RobotMap.ARM_ELBOW_ROTATION_MOTOR_VICTOR);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());  

    elbowRotationMain.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		elbowRotationMain.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		elbowRotationMain.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		elbowRotationMain.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    elbowRotationMain.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    elbowRotationMain.configMotionCruiseVelocity(ELBOW_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    elbowRotationMain.configMotionAcceleration(ELBOW_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);

    /* Config closed loop gains for Primary closed loop (Current) */
    elbowRotationMain.config_kP(CURRENT_CONTROL_SLOT, currentP, RobotConstants.kTimeoutMs);
    elbowRotationMain.config_kI(CURRENT_CONTROL_SLOT, currentI, RobotConstants.kTimeoutMs);
    elbowRotationMain.config_kD(CURRENT_CONTROL_SLOT, currentD, RobotConstants.kTimeoutMs);
    elbowRotationMain.config_kF(CURRENT_CONTROL_SLOT, currentF, RobotConstants.kTimeoutMs);

    elbowRotationMain.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    elbowRotationMain.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    elbowRotationMain.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    elbowRotationMain.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    elbowRotationMain.setNeutralMode(NeutralMode.Brake);
    elbowRotationSlave.setNeutralMode(NeutralMode.Brake);

    elbowRotationMain.configClosedloopRamp(0);
    elbowRotationMain.configOpenloopRamp(RAMP_RATE);

    // These thresholds stop the motor when limit is reached
    elbowRotationMain.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    elbowRotationMain.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
    elbowRotationMain.configContinuousCurrentLimit(NORMAL_CONTINUOUS_CURRENT_LIMIT);

    // Enable Safety Measures
    elbowRotationMain.configForwardSoftLimitEnable(true);
    elbowRotationMain.configReverseSoftLimitEnable(true);
    elbowRotationMain.enableCurrentLimit(true);
    elbowRotationMain.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);

    // Sets the Quadrature Position based on the pulse width to work as an absolute encoder 
    elbowRotationMain.getSensorCollection().setQuadraturePosition(elbowRotationMain.getSensorCollection().getPulseWidthPosition() * 4, 0); // Times 4 since Quadrature is out of 4096 while Pulse Width is 1024
 
    setpoint = NO_SETPOINT;

    int pulseWidthPos = getElbowPulseWidth()&4095;

    if (pulseWidthPos < QUAD_ENCODER_MIN) {
      pulseWidthPos += 4096;
    } 
    if (pulseWidthPos > QUAD_ENCODER_MAX) {
      pulseWidthPos -= 4096;
    }

    elbowRotationMain.getSensorCollection().setQuadraturePosition(pulseWidthPos, 0);

    elbowRotationMain.configContinuousCurrentLimit(NORMAL_CONTINUOUS_CURRENT_LIMIT);
    elbowRotationMain.configPeakCurrentLimit(0);
    elbowRotationMain.enableCurrentLimit(true);

    enablePercentOutput();
 
  }

  @Override
  public void setOutput(double output) {
    elbowRotationMain.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getOutputCurrent() {
    return elbowRotationMain.getOutputCurrent();
  }

  @Override
  public void setClimbingCurrentLimit() {
    elbowRotationMain.configContinuousCurrentLimit(CLIMBING_CONTINUOUS_CURRENT_LIMIT);
  }

  @Override
  public void setNormalCurrentLimit() {
    elbowRotationMain.configContinuousCurrentLimit(NORMAL_CONTINUOUS_CURRENT_LIMIT);
  }

  @Override
  public int getPositionTicks() {
    return elbowRotationMain.getSensorCollection().getQuadraturePosition();
  }
  
  @Override
  public double getAngle() {
    return convertElbowTicksToDegrees(getPositionTicks());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickElbow(this));
  }

  @Override
  public boolean isForwardLimitPressed() {
    //drive until switch is closed
    return elbowRotationMain.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public boolean isReverseLmitPressed() {
    //drive until switch is closed
    return elbowRotationMain.getSensorCollection().isRevLimitSwitchClosed();
  }
  
  @Override
  public void zero(double encoderValue) {
    elbowRotationMain.getSensorCollection().setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  @Override
  public double getEncoderValue() {
    return elbowRotationMain.getSensorCollection().getQuadraturePosition();
  }

  @Override
  public void setMotionMagicSetpointTicks(int elbowSetpointInTicks) {
    super.setpoint = elbowSetpointInTicks;
    elbowRotationMain.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
    elbowRotationMain.set(ControlMode.MotionMagic, elbowSetpointInTicks);
  }

  @Override
  public void setCurrentControl(int current) {
    elbowRotationMain.selectProfileSlot(CURRENT_CONTROL_SLOT, 0);
    elbowRotationMain.set(ControlMode.Current, current);
  }

  public static int convertElbowDegreesToTicks(double degrees) {
    // If straight up is 0 and going forward is positive
    // percentage * half rotation
    return (int)((degrees / 180) * (0.5 * RobotConstants.ELBOW_TICKS_PER_ROTATION));
}

public static double convertElbowTicksToDegrees(double ticks) {
   //If straight up is 0 and going forward is positive
   // percentage * half degrees rotation
    return (ticks / (0.5 * RobotConstants.ELBOW_TICKS_PER_ROTATION)) * 180;
}

@Override
public void setMotionMagicSetpoint(double elbowAngle) {
  int ticks = (int) convertElbowDegreesToTicks(elbowAngle);
  elbowRotationMain.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
  elbowRotationMain.set(ControlMode.MotionMagic, ticks);
}

@Override
public void updateArbitraryFeedForward() {
  if(setpoint != NO_SETPOINT) {
    double value = -1.0 * Math.cos(Math.toRadians(getAngle())) * ARBITRARY_FEEDFORWARD_CONSTANT;
    elbowRotationMain.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, value);
  }
}

public int getElbowPulseWidth() {  
  return elbowRotationMain.getSensorCollection().getPulseWidthPosition();
}


}
