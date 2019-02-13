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
import frc.team670.robot.commands.climb.pistonClimb.JoystickPistonClimb;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.DPadState;

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
    if (controller.getAButton() && controller.getBButton() && controller.getXButton() && controller.getYButton()) {
      os = OperatorState.ARM;
    } else if (controller.getStartButton() && controller.getBackButton()) {
      os = OperatorState.CLIMB;
    }

    if (os == OperatorState.ARM) {
      Scheduler.getInstance().add(new JoystickElbow(Robot.arm.getElbow(), controller.getRightStickY()));
      Scheduler.getInstance().add(new JoystickWrist(Robot.arm.getWrist(), controller.getLeftStickY()));

      double extensionPower = Math.pow((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()), 2);
      Scheduler.getInstance().add(new JoystickExtension(Robot.arm, Robot.intake, Robot.arm.getExtension(), extensionPower));
    } else if (os == OperatorState.CLIMB) {
      Scheduler.getInstance().add(new JoystickPistonClimb(Robot.climber, controller.getLeftStickY(), controller.getRightStickY()));
    }

    if (controller.getRightBumper()) {
      Scheduler.getInstance().add(new RunIntake(Robot.intake, Robot.sensors, true));
    } else if (controller.getLeftBumper()) {
      Scheduler.getInstance().add(new RunIntake(Robot.intake, Robot.sensors, false));
    }
  }

  /**
   * An enum determining the state of the controler
   */
  public enum OperatorState {
    ARM, CLIMB, NONE
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

}