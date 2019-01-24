/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.armClimb;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Extension;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Command to run the "dragging forward motion" of the arm to get it onto the
 * platform
 * 
 */
public class ArmClimb extends Command {
  private double heightInInches;
  private static boolean userWishesToStillClimb;
  private int loggingIterationCounter;

  public static final int CLIMB_CURRENT = 10; // TODO figure required limited current

  public ArmClimb() {
    super();
    requires(Robot.arm);
    requires(Robot.elbow);
    userWishesToStillClimb = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Set arm to ready to climb state
    // telescope out

    Robot.extension.enableExtensionPIDController();

    heightInInches = Robot.climber.getFrontTalonPositionInInches() + RobotConstants.ARM_HEIGHT_IN_INCHES + RobotConstants.DRIVEBASE_TO_GROUND; // TODO get the actual method

    Logger.consoleLog("startHeightOfRobot%s startAngleOfElbow%s ", heightInInches, Robot.elbow.getElbowAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    holdElbowDownWithCurrentLimit(CLIMB_CURRENT); // Brings arm down

    double deltaSetPointInInches = (heightInInches/(Math.cos(Math.toDegrees(Robot.elbow.getElbowAngle())))) - RobotConstants.FIXED_ARM_LENGTH_IN_INCHES;

    int deltaSetPointInTicks = MathUtils.convertExtensionInchesToTicks(deltaSetPointInInches);

    Robot.extension.setPIDControllerSetpoint(Extension.EXTENSION_ENCODER_OUT - deltaSetPointInTicks); // Changes the
                                                                                                    // setpoint
    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("heightOfRobot%s angleOfElbow%s extensionSetpoint%s ", heightInInches, Robot.elbow.getElbowAngle(), Extension.EXTENSION_ENCODER_OUT - deltaSetPointInTicks);

    loggingIterationCounter++;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Is only set to false when CancelArmClimb command is called
    if (!userWishesToStillClimb)
      return true;

    // If arm pulls in too much and no longer makes contact with surface
    if (Robot.extension.getExtensionLengthInTicks() < (Math.cos(Robot.elbow.getElbowAngle() / heightInInches))) {
      return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    releaseElbow();
    Logger.consoleLog("endHeightOfRobot%s endAngleOfElbow%s ", heightInInches, Robot.elbow.getElbowAngle());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog("ArmClimb interrupted");
  }

  /**
   * Holds the elbow down by setting a current limit and running the motor down at
   * full speed
   * 
   */
  private void holdElbowDownWithCurrentLimit(int currentLimit) {
    Robot.elbow.setClimbingCurrentLimit();
    //Climb current defaults to negative because of the way arm is flipped during climbing
    Robot.elbow.getElbowTalon().set(ControlMode.Current, -CLIMB_CURRENT);
  }

  /**
   * Releases the elbow by setting the current limit back to normal and setting
   * the motor power to 0
   */
  private void releaseElbow() {
    Robot.elbow.setOutput(0);
    Robot.elbow.setNormalCurrentLimit();
  }

  /**
   * Returns a boolean wish keeps track of whether or not the cancel command has
   * been called
   */
  public static boolean getUserWishesToStillClimb() {
    return userWishesToStillClimb;
  }

  /**
   * Sets whether or not the user has canceled the command to use arm climb
   */
  public static void setUserWishesToStillClimb(boolean userWishesToStillClimb) {
    ArmClimb.userWishesToStillClimb = userWishesToStillClimb;
  }

}
