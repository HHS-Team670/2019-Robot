/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.arm.movement.MoveElbow;
import frc.team670.robot.commands.arm.movement.MoveExtension;
import frc.team670.robot.commands.arm.movement.MoveWrist;
import frc.team670.robot.commands.tuning.DecreaseMeasurementOutput;
import frc.team670.robot.commands.tuning.IncreaseMeasurementOutput;
import frc.team670.robot.commands.tuning.MeasureArbitraryFeedforward;
import frc.team670.robot.commands.tuning.MoveRotatorToSetpoint;
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
  private Joystick rightStick, arcadeButtons;
  private MustangController driverController, operatorController;

  private XKeys xkeys;

  // Buttons
  private JoystickButton toggleReverseDrive;
  private JoystickButton flipCameras;
  private JoystickButton flipArmDriverControlState;

  private JoystickButton incFeedForward, decFeedForward, measureFeedForward, runForward, runBackward;
  private JoystickButton runIntakeIn, runIntakeOut;


  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys();
    
    // toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    // toggleReverseDrive.whenPressed(new FlipDriveDirection());
    // flipCameras = new JoystickButton(driverController, XboxButtons.B);
    // flipCameras.whenPressed(new FlipCamera());
    // flipArmDriverControlState = new JoystickButton(operatorController, XboxButtons.RIGHT_JOYSTICK_BUTTON);
    // flipArmDriverControlState.whenPressed(new FlipJoystickArmControl());

    incFeedForward = new JoystickButton(driverController, XboxButtons.START);
    incFeedForward.whenPressed(new IncreaseMeasurementOutput());
    decFeedForward = new JoystickButton(driverController, XboxButtons.BACK);
    decFeedForward.whenPressed(new DecreaseMeasurementOutput());
    measureFeedForward = new JoystickButton(driverController, XboxButtons.X);
    measureFeedForward.whenPressed(new MeasureArbitraryFeedforward(Robot.arm.getElbow()));

    runForward = new JoystickButton(driverController, XboxButtons.B);
    runForward.whenPressed(new MoveElbow(Robot.arm.getElbow(), -90));
    runBackward = new JoystickButton(driverController, XboxButtons.A);
    runBackward.whenPressed(new MoveElbow(Robot.arm.getElbow(), 90));

    // runIntakeIn = new JoystickButton(operatorController, XboxButtons.RIGHT_BUMPER);
    // runIntakeIn.whenPressed(new ButtonRunIntake(Robot.intake, RunIntakeInWithIR.RUNNING_POWER, true));
    // runIntakeIn.whenReleased(new StopIntakeRollers(Robot.intake));
    // runIntakeOut = new JoystickButton(operatorController, XboxButtons.LEFT_BUMPER);
    // runIntakeOut.whenPressed(new ButtonRunIntake(Robot.intake, RunIntakeInWithIR.RUNNING_POWER, false));
    // runIntakeOut.whenReleased(new StopIntakeRollers(Robot.intake));
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
