/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.arm.joystick.JoystickElbow;
import frc.team670.robot.commands.arm.joystick.JoystickExtension;
import frc.team670.robot.commands.arm.joystick.JoystickWrist;
import frc.team670.robot.utils.MustangController;

/**
*
*/
public class ControlOperatorController extends Command {
  private OperatorState os;
  private MustangController controller;

  public ControlOperatorController(MustangController controller) {
    super();
    os = OperatorState.NONE;
    this.controller = controller;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    
  }

  @Override
  protected void execute() {
    // Check for states
    if (controller.getAButton() && controller.getBButton() && controller.getXButton() && controller.getYButton()) {
      os = OperatorState.ARM;
    } 

    // If in arm state
    if (os == OperatorState.ARM) {
      Scheduler.getInstance().add(new JoystickElbow(Robot.arm.getElbow(), Math.pow(controller.getRightStickY(), 2)));
      Scheduler.getInstance().add(new JoystickWrist(Robot.arm.getWrist(),  Math.pow(controller.getLeftStickY(), 2)));

      double extensionPower = Math.pow((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()), 2);
      Scheduler.getInstance().add(new JoystickExtension(Robot.arm, Robot.intake, Robot.arm.getExtension(), extensionPower));
    } 
    
    // // If in climb state
    // else if (os == OperatorState.CLIMB) {
    //   Scheduler.getInstance().add(new JoystickPistonClimb(Robot.climber, controller.getLeftStickY() * controller.getLeftStickY(), controller.getRightStickY() * controller.getRightStickY()));
    // }
  }

  /**
   * An enum determining the state of the controler
   */
  public enum OperatorState {
    ARM, NONE
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

}