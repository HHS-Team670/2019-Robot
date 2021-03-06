/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.utils.Logger;

public class TimedRunIntake extends TimedCommand {

  private BaseIntake intake;
  private boolean runningIn;
  private double power;

  /**
   * 
   * @param secondsToRun the time for the intake to run in seconds
   */
  public TimedRunIntake(double secondsToRun, double power, BaseIntake intake, boolean runningIn) {
    super(secondsToRun);
    requires(intake);
    this.intake = intake;
    this.power = power;
    this.runningIn = runningIn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intake.runIntake(power, runningIn);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.runIntake(0, true);
    XKeys.setBothToggles(false);
    Logger.consoleLog();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.runIntake(0, true);
    Logger.consoleLog();
  }
}
