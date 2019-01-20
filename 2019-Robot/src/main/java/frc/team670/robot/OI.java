/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team670.robot.commands.ZeroNavX;
import frc.team670.robot.commands.drive.FlipDriveDirection;
import frc.team670.robot.commands.drive.XboxRocketLeagueDrive;
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
  private Joystick rightStick, operatorStick, arcadeButtons;
  private MustangController driverController;
  private JoystickButton resetNavX;

  // Buttons
  private JoystickButton toggleReverseDrive;


  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveDirection());
    resetNavX = new JoystickButton(driverController, XboxButtons.A);
    resetNavX.whenPressed(new ZeroNavX());
  }

  public MustangController getDriverController() {
    return driverController;
  }

  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }
}
