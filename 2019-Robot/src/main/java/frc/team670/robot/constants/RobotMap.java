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
      //Intake
  public static final int INTAKE_BASE_ENCODER_CHANNEL_A = 0; //TODO Set these
  public static final int INTAKE_BASE_ENCODER_CHANNEL_B = 0;

  // Arm
  public static final int ARM_TRANSLATION_MOTOR = 4;
  public static final int ARM_EXTENSION_MOTOR = 5;
  public static final int ARM_ELBOW_ROTATION_MOTOR_TALON = 6;
  public static final int ARM_ELBOW_ROTATION_MOTOR_VICTOR = 7;
  public static final int ARM_WRIST_ROTATION = 0;

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
  public final static int INTAKE_IR_DIO_PORT = 0; // TODO set this
  public final static int CLAW_IR_DIO_PORT = 0; // TODO set this

  // Joysticks
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  //Intake
  public static final int INTAKE_BASE_TALON = 0;//8; // TODO Set these
  public static final int INTAKE_ROLLER_VICTOR = 8;


}
