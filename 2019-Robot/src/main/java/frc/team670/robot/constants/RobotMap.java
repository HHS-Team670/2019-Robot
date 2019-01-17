/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // Drive Base
  public static final int sparkLeftMotor1 = 0;
  public static final int sparkLeftMotor2 = 1;
  public static final int sparkRightMotor1 = 2;
  public static final int sparkRightMotor2 = 3;

  // Arm
  public static final int armTranslationMotor = 4;
  public static final int armExtensionMotor = 5;
  public static final int armElbowRotationMotorTalon = 6;
  public static final int armElbowRotationMotorVictor = 7;
  public static final int armWristRotation = 8;
  //Encoders
  public static final int leftEncoderChannelA = 0;
  public static final int leftEncoderChannelB = 1;
  public static final int rightEncoderChannelA = 2;
  public static final int rightEncoderChannelB = 3;
  
  // Sensor Ports
  public final static Port navXPort = SerialPort.Port.kUSB;

  // Joysticks
  public static final int DRIVER_CONTROLLER_PORT = 0;

}
