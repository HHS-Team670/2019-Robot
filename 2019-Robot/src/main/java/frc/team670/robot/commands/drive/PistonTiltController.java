/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.dataCollection.MustangSensors;


public class PistonTiltController extends Command {

  private double navXPitch;
  private double tiltControllerLowerOutput = 0.5, tiltControllerUpperOutput = 1.5;
  private double tiltControllerOutput;


  private Climber climber;

  private PIDController tiltController;


  public PistonTiltController() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);

    tiltController.setOutputRange(tiltControllerLowerOutput, tiltControllerUpperOutput);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.sensors.getPitchDouble() < 0) {

      climber.getFrontController().setOutputRange(minimumOutput, maximumOutput);
      // getFrontController.setOutputRange(frontOutputStandard * pitchOutput)

    } else if (Robot.sensors.getPitchDouble() > 0) {

      tiltControllerOutput = tiltController.get() - 1;

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
