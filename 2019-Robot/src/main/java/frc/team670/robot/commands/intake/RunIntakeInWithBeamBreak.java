/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.dataCollection.XKeys;

public class RunIntakeInWithBeamBreak extends Command {

  private BaseIntake intake;
  private MustangSensors sensors;

  private boolean hasBeenTriggered;

  public RunIntakeInWithBeamBreak(BaseIntake intake, MustangSensors sensors) {
    requires(intake);
    this.intake = intake;
    this.sensors = sensors;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "RunIntakeInWithIR");
    hasBeenTriggered = false;
    Logger.consoleLog();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
    // If the IR sensor has been tripped and it is for the first time
    if (!sensors.isIntakeBeamBreakNull() && !sensors.getIntakeBeamBreakOutput() && !hasBeenTriggered) {
      hasBeenTriggered = true;
      intake.runIntakeUsingCurrent(Intake.INTAKE_RUNNING_CURRENT/2);
      setTimeout(0.3 + timeSinceInitialized());
    }else{
      intake.runIntakeUsingCurrent(Intake.INTAKE_RUNNING_CURRENT);
    }
    // SmartDashboard.putBoolean("BeamBrake Triggered", hasBeenTriggered); 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // If 0.5 seconds has passed since the IR sensor was first tripped
    if (!sensors.isIntakeBeamBreakNull()) {
      return (isTimedOut());
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.runIntake(0, true);
    Logger.consoleLog();
    XKeys.setBothToggles(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.runIntake(0, true);
    Logger.consoleLog();
    end();
  }
}
