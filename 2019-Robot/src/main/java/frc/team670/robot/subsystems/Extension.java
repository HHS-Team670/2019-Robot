/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.SettingUtils;

/**
 * Controls motors for motion of extension
 */
public class Extension extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX extensionMotor;

  private double P = 0.0, I = 0.0, D = 0.0, F = 0.0;

  public static final int EXTENSION_ENCODER_OUT = 0;

  private static final int POSITION_SLOT = 0;
  private static final double EXTENSION_POWER = 0.75;

  public Extension() {
    extensionMotor = new TalonSRX(RobotMap.ARM_EXTENSION_MOTOR);
  }

  /**
   * Enables the PID Controller for extension
   * 
   */
  public void enableExtensionPIDController() {
    SettingUtils.initTalonPID(extensionMotor, POSITION_SLOT, P, I, D, F, -EXTENSION_POWER, EXTENSION_POWER);
  }

  /**
   * Modifies the setpoint for the PID Controller
   */
  public void setPIDControllerSetpoint(int setpoint) {
    extensionMotor.set(ControlMode.Position, setpoint);
  }

  /**
   * Returns the length of the extension in ticks
   */
  public int getExtensionLengthInTicks() {
    return extensionMotor.getSensorCollection().getQuadraturePosition();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean getReverseLimitSwitch() {
    //drive until switch is closed
    return extensionMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

}
