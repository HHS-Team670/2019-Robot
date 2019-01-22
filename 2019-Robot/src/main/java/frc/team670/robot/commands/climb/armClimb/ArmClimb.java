/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.armClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;

public class ArmClimb extends Command {
  private static final double elbowOutput = 1.0; // Might have to make this negative depending on how motors are
                                                 // oriented.
  private double height;

  private static boolean canClimb;

  public ArmClimb() {
    super();
    requires(Robot.arm);
    requires(Robot.elbow);
    canClimb = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Set arm to ready to climb state
    // telescope out

    Robot.extension.enableExtensionPIDController();

    height = Robot.climber.getFrontTalonPositionInInches() + RobotConstants.ARM_HEIGHT; // TODO get the actual method
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!canClimb) {
      super.cancel();
    }

    holdElbowDownWithCurrentLimit(RobotConstants.CLIMB_CURRENT_LIMIT); // Brings arm down

    int deltaSetPoint = (int) (height
        / (Math.toDegrees(Math.cos(Robot.elbow.getElbowAngle()) - RobotConstants.FIXED_ARM_LENGTH))); // need to convert
                                                                                                      // this to ticks

    Robot.extension.setPIDControllerSetpoint(RobotConstants.EXTENSION_ENCODER_OUT - deltaSetPoint);

    if (Robot.extension.getExtensionLengthInTicks() < (Math.cos(Robot.elbow.getElbowAngle() / height))) {
      super.cancel();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !canClimb;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    releaseElbow(RobotConstants.NORMAL_CURRENT_LIMIT);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  /**
   * Holds the elbow down by setting a current limit and running the motor down at full speed
   * 
   */
  private void holdElbowDownWithCurrentLimit(int currentLimit) {
    Robot.elbow.setCurrentLimit(currentLimit);
    Robot.elbow.enableCurrentLimit();
    Robot.elbow.setOutput(elbowOutput);
  }


  /**
   * Releases the elbow by setting the current limit back to normal and setting the motor power to 0 
   */
  private void releaseElbow(int currentLimit) {
    Robot.elbow.setCurrentLimit(currentLimit);
    Robot.elbow.disableCurrentLimit();
    Robot.elbow.setOutput(0);
  }

  public static boolean canClimb() {
    return canClimb;
  }

  public static void setCanClimb(boolean canClimb) {
    ArmClimb.canClimb = canClimb;
  }

}
