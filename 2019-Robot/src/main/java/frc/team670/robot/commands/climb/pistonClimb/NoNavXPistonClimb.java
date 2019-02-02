/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.functions.SettingUtils;

/**
 * Class to deploy the pistons down with a some degree of control from the encoders
 * 
 */
public class NoNavXPistonClimb extends Command {

  private double tiltControllerLowerOutput = -0.2, tiltControllerUpperOutput = 0.2;
  private double tiltTolerance = 0; // TODO set this
  private double P = 0.1, I = 0.0, D = 0.0, F = 0.0; // TODO set these

  private PIDController tiltController;
  private boolean goingUp;
  private int loggingIterationCounter, setPoint;

  private Climber climber;

  /**
   * @param setPoint the desired end goal of the climber (Flat, Level 2 or Level
   *                 3)
   * @param climber  the climber upon which this command will be called on
   */
  public NoNavXPistonClimb(int setPoint, Climber climber) {
    requires(climber);
    
    this.setPoint = setPoint;
    this.climber = climber;
    tiltController = new PIDController(P, I, D, F, new ClimberEncoderPitchPIDSource(climber), new NullPIDOutput());
    tiltController.setSetpoint(0);
    tiltController.setAbsoluteTolerance(tiltTolerance);
    tiltController.setOutputRange(tiltControllerLowerOutput, tiltControllerUpperOutput);
    tiltController.enable();
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    climber.enableClimberPIDControllers(setPoint);

    goingUp = (setPoint >= climber.getFrontTalonPositionInTicks());

    Logger.consoleLog("startBackPistonPosition:%s startFrontPistonPosition:%s ", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if (Math.abs(climber.getPitchFromEncoders() - tiltTolerance) > 0) {
          climber.handleTilt(goingUp, tiltTolerance, tiltController.get());
      }

    Logger.consoleLog("currentBackPistonPosition:%s, currentFrontPistonPosition:%s, tilt:%s", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks(), climber.getPitchFromEncoders());
    

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.getFrontControllerOnTarget() && climber.getBackControllerOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SettingUtils.releaseController(tiltController);
    Logger.consoleLog("endBackPistonPosition:%s, endFrontPistonPosition:%s ", climber.getBackTalonPositionInTicks(), climber.getFrontTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog();
  }

  public class ClimberEncoderPitchPIDSource implements PIDSource {
    private Climber climber;
    private PIDSourceType pidSourceType;

    public ClimberEncoderPitchPIDSource(Climber climber) {
      this.climber = climber;
      pidSourceType = PIDSourceType.kDisplacement;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return pidSourceType;
    }

    @Override
    public double pidGet() {
      return climber.getPitchFromEncoders();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
      this.pidSourceType = pidSource;
    }
  }
}
