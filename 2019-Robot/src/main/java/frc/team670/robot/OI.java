/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team670.robot.commands.intake.MoveIntakeToSetpointAngle;
import frc.team670.robot.commands.tuning.DecreaseMeasurementOutput;
import frc.team670.robot.commands.tuning.IncreaseMeasurementOutput;
import frc.team670.robot.commands.tuning.MeasureArbitraryFeedforward;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  // Controllers/Joysticks
  private Joystick rightStick, arcadeButtons;
  private MustangController driverController, operatorController;
  private JoystickButton resetNavX;

  // Buttons
  private JoystickButton toggleReverseDrive;
  private JoystickButton flipCameras;
  private JoystickButton flipArmDriverControlState;

  private JoystickButton incFeedForward, decFeedForward, measureFeedForward, runForward, runBackward;


  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    
    // toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    // toggleReverseDrive.whenPressed(new FlipDriveDirection());
    // flipCameras = new JoystickButton(driverController, XboxButtons.B);
    // flipCameras.whenPressed(new FlipCamera());
    // flipArmDriverControlState = new JoystickButton(operatorController, XboxButtons.RIGHT_JOYSTICK_BUTTON);
    // flipArmDriverControlState.whenPressed(new FlipJoystickArmControl());
    // resetNavX = new JoystickButton(driverController, XboxButtons.A);
    // resetNavX.whenPressed(new ZeroNavX());

    incFeedForward = new JoystickButton(driverController, XboxButtons.START);
    incFeedForward.whenPressed(new IncreaseMeasurementOutput());
    decFeedForward = new JoystickButton(driverController, XboxButtons.BACK);
    decFeedForward.whenPressed(new DecreaseMeasurementOutput());
    measureFeedForward = new JoystickButton(driverController, XboxButtons.X);
    measureFeedForward.whenPressed(new MeasureArbitraryFeedforward(Robot.intake));

    runForward = new JoystickButton(driverController, XboxButtons.B);
    runForward.whenPressed(new MoveIntakeToSetpointAngle(80, Robot.intake));
    // runBackward = new JoystickButton(driverController, XboxButtons.A);
    // runBackward.whenPressed(new MoveIntakeToSetpointAngle(-80, Robot.intake));
  }

  public MustangController getDriverController() {
    return driverController;
  }

  public MustangController getOperatorController() {
    return operatorController;
  }

  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }
}
