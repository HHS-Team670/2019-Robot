/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.cameras.FlipCamera;
import frc.team670.robot.commands.claw.CloseClaw;
import frc.team670.robot.commands.claw.OpenClaw;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.commands.intake.MoveIntakeToSetpointAngle;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Controllers/Joysticks
  // private Joystick rightStick, arcadeButtons;
  private MustangController driverController, operatorController;

  private XKeys xkeys;

  // Buttons
  private JoystickButton toggleReverseDrive;
  private JoystickButton flipCameras;

  private JoystickButton incFeedForward, decFeedForward, measureFeedForward, runForward, runBackward;
  private JoystickButton enableBrakeMode, enableCoastMode;

  private JoystickButton openClaw, closeClaw, intakeOut, intakeIn;


  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    // operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys();
    
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
    flipCameras = new JoystickButton(driverController, XboxButtons.B);
    flipCameras.whenPressed(new FlipCamera());

    openClaw = new JoystickButton(driverController, XboxButtons.Y);
    openClaw.whenPressed(new OpenClaw(Robot.claw));
    closeClaw = new JoystickButton(driverController, XboxButtons.X);
    closeClaw.whenPressed(new CloseClaw(Robot.claw));

    intakeIn = new JoystickButton(driverController, XboxButtons.START);
    intakeIn.whenPressed(new MoveIntakeToSetpointAngle(70, Robot.intake));
    intakeOut = new JoystickButton(driverController, XboxButtons.BACK);
    intakeOut.whenPressed(new MoveIntakeToSetpointAngle(-70, Robot.intake));


    // incFeedForward = new JoystickButton(driverController, XboxButtons.START);
    // incFeedForward.whenPressed(new IncreaseMeasurementOutput());
    // decFeedForward = new JoystickButton(driverController, XboxButtons.BACK);
    // decFeedForward.whenPressed(new DecreaseMeasurementOutput());
    // measureFeedForward = new JoystickButton(driverController, XboxButtons.X);
    // measureFeedForward.whenPressed(new MeasureArbitraryFeedforward(Robot.arm.getWrist()));

    // runForward = new JoystickButton(driverController, XboxButtons.B);
    // runForward.whenPressed(new MoveElbow(Robot.arm.getElbow(), -45));
    // runBackward = new JoystickButton(driverController, XboxButtons.A);
    // runBackward.whenPressed(new MoveElbow(Robot.arm.getElbow(), 45));



    // enableBrakeMode = new JoystickButton(operatorController, XboxButtons.RIGHT_BUMPER);
    // enableBrakeMode.whenPressed(new EnableArmBrakeMode(Robot.arm));
    // enableCoastMode = new JoystickButton(operatorController, XboxButtons.LEFT_BUMPER);
    // enableCoastMode.whenPressed(new EnableArmCoastMode(Robot.arm));
  }

  /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  /**
   * Sets the rumble on the operator controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleOperatorController(double power, double time) {
    rumbleController(operatorController, power, time);
  }

  private void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
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

  public Command getSelectedAutonCommand() {
    SmartDashboard.putBoolean("auto-command", xkeys.getAutonCommand() == null);
    return xkeys.getAutonCommand();
  }
}
