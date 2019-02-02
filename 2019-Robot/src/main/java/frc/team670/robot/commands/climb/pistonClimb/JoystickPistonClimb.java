/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MustangController;

/**
 * Runs the piston climb with a percent output based on input from the joystick
 * 
 */
public class JoystickPistonClimb extends Command {
  private Climber climber;
  private MustangController controller;
  private int tolerance = 500;
  private int loggingIterationCounter;

  /**
   * @param climber    the climber being used
   * @param controller the operator controller being used
   */
  public JoystickPistonClimb(Climber climber, MustangController controller) {
    this.climber = climber;
    this.controller = controller;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog("startBackPistonPosition:%s startFrontPistonPosition:%s ", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double frontPower = controller.getLeftStickY();
    frontPower *= (frontPower > 0) ? Climber.MAXIMUM_PISTON_POWER : -1 * Climber.MINIMUM_PISTON_POWER; //Multiplied by -1 because constant is negative and so is stick input
    double backPower = controller.getRightStickY();
    backPower *= (backPower > 0) ? Climber.MAXIMUM_PISTON_POWER : -1 * Climber.MINIMUM_PISTON_POWER; //Multiplied by -1 because constant is negative and so is stick input

    // Front pistons are approaching flat position so either the entire robot is
    // almost back down or the front pistons are almost retracted. If the original
    // input is already at -0.05 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (frontPower < -0.05 && climber.getFrontTalonPositionInTicks() <= Climber.PISTON_ENCODER_FLAT + tolerance) {
      frontPower = Math.min(-0.05, (frontPower * (Math.abs(climber.getFrontTalonPositionInTicks() - (Climber.PISTON_ENCODER_FLAT + tolerance)) / tolerance)));
    }
    // Front pistons are approaching fully deployed position. If the original input
    // is already at 0.1 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (frontPower > 0.1 && climber.getFrontTalonPositionInTicks() >= Climber.PISTON_ENCODER_LEVEL_THREE - tolerance) {
      frontPower *= Math.max(0.1, (frontPower * (Climber.PISTON_ENCODER_LEVEL_THREE - climber.getFrontTalonPositionInTicks()) / tolerance));
    }

    // Back pistons are approaching flat position so either the entire robot is
    // almost back down or the back pistons are almost retracted
    if (backPower < -0.05 && climber.getBackTalonPositionInTicks() <= Climber.PISTON_ENCODER_FLAT + tolerance) {
      backPower = Math.min(-0.05, (backPower * (Math.abs(climber.getFrontTalonPositionInTicks() - (Climber.PISTON_ENCODER_FLAT + tolerance)) / tolerance)));
    }

    // Back pistons are approaching fully deployed position. If the original input
    // is already at 0.1 (arbitrary value)
    // or lower magnitude, there's no need to limit it even further
    if (backPower > 0.1 && climber.getBackTalonPositionInTicks() >= Climber.PISTON_ENCODER_LEVEL_THREE - tolerance) {
      backPower *= Math.max(0.1, (backPower * (Climber.PISTON_ENCODER_LEVEL_THREE - climber.getFrontTalonPositionInTicks()) / tolerance));
    }

    climber.drivePistons(frontPower, backPower);

    Logger.consoleLog("currentBackPistonPosition:%s currentFrontPistonPosition:%s", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.drivePistons(0, 0);
    climber.enableClimberPIDControllers(climber.getFrontTalonPositionInTicks());
    Logger.consoleLog("endBackPistonPosition:%s, endFrontPistonPosition:%s ", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
    end();
  }
}
