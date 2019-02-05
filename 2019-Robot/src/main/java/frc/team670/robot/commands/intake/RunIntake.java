/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.utils.Logger;

public class RunIntake extends Command {

  private BaseIntake intake;
  private MustangSensors sensors;
  private boolean runningIn;

  private static final double RUNNING_POWER = 0.75; // TODO figure out if we want to run full speed
  private boolean hasBeenTriggered;
  private long timeSinceIRSensorTripped, timeSinceInitialized;

  public RunIntake(BaseIntake intake, MustangSensors sensors, boolean runningIn) {
    requires(intake);
    this.intake = intake;
    this.sensors = sensors;
    this.runningIn = runningIn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog();
    timeSinceInitialized = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intake.runIntake(RUNNING_POWER, runningIn);

      // If the IR sensor has been tripped and it is for the first time
      if (sensors.getIntakeIRSensor() != null && sensors.getIntakeIROutput() && !hasBeenTriggered) {
        hasBeenTriggered = true;
        timeSinceIRSensorTripped = System.currentTimeMillis();
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(sensors.getIntakeIRSensor() == null || sensors.getClawIRSensor() == null){
      return (System.currentTimeMillis() - timeSinceInitialized > 500);
    }
    // If 0.5 seconds has passed since the IR sensor was first tripped or if the
    // cargo is already in the claw
    return (System.currentTimeMillis() - timeSinceIRSensorTripped > 500 || sensors.getClawIROutput());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.runIntake(0, true);
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
