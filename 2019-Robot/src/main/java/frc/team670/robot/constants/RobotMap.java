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

  // TODO set all of these.

  // Do not put Talon or other CTRE things to CAN ID 0 because that's the PDP. CAN conflicts are only within device families.

  public static int PDP_ID = 0;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 0; // These are properly set.
  public static final int SPARK_LEFT_MOTOR_2 = 1;
  public static final int SPARK_RIGHT_MOTOR_1 = 2;
  public static final int SPARK_RIGHT_MOTOR_2 = 3;

  //Encoders
      //Drivebase
  public static final int LEFT_ENCODER_CHANNEL_A = 0; // These are properly set
  public static final int LEFT_ENCODER_CHANNEL_B = 1;
  public static final int RIGHT_ENCODER_CHANNEL_A = 2;
  public static final int RIGHT_ENCODER_CHANNEL_B = 3;

  // Arm
  public static final int ARM_EXTENSION_MOTOR = 8;
  public static final int ARM_ELBOW_ROTATION_MOTOR_TALON = 6;
  public static final int ARM_ELBOW_ROTATION_MOTOR_VICTOR = 5;
  public static final int ARM_WRIST_ROTATION = 9;

  // Claw
  public static final int HARD_GRIP_SOLENOID = 9;
  public static final int SOFT_GRIP_SOLENOID = 10;
  public static final int CLAW_PUSH_SOLENOID = 11;

  public static final int PC_MODULE = 12;

  // Climber
  public static final int BACK_CLIMBER_PISTON_CONTROLLER = 14;
  public static final int FRONT_CLIMBER_PISTON_CONTROLLER = 15;
  
  // Sensors
  public final static Port NAVX_PORT = SerialPort.Port.kUSB;  

  public static final int FRONT_ULTRASONIC_TRIGGER_PIN = 7; // TODO set these
  public static final int FRONT_ULTRASONIC_ECHO_PIN = 8;
  public static final int BACK_LEFT_ULTRASONIC_TRIGGER_PIN = 9;
  public static final int BACK_LEFT_ULTRASONIC_ECHO_PIN = 10;
  public static final int BACK_RIGHT_ULTRASONIC_TRIGGER_PIN = 11;
  public static final int BACK_RIGHT_ULTRASONIC_ECHO_PIN = 12;
  public final static int INTAKE_IR_DIO_PORT = 13;


  // Joysticks
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  //Intake
  public static final int INTAKE_BASE_TALON = 7;
  public static final int INTAKE_ROLLER_TALON = 4;


}
