/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.armClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.utils.Logger;

/**
 * Command to run the "dragging forward motion" of the arm to get it onto the
 * platform. Assumes the arm is already in the "ReadyToClimb" ArmState
 * 
 */
public class ArmClimb extends Command {
  private double heightInInches;
  private static boolean userWishesToStillClimb;
  private int loggingIterationCounter;

  private BaseExtension extension;
  private BaseElbow elbow;

  private Climber climber;
  private Arm arm;

  public static final int CLIMB_CURRENT = -10; // TODO figure required limited current. Negative to move backwards
  public static final double LEVEL_THREE_PLATFORM_HEIGHT_IN_INCHES = 19;

  /**
   * Prepares the ArmClimb method allowing the arm to drag the robot forward.
   */
  public ArmClimb(Arm arm, Climber climber) {
    super();
    extension = arm.getExtension();
    elbow = arm.getElbow();
    requires(elbow);
    requires(extension);
    userWishesToStillClimb = true;
    this.arm = arm;
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    extension.enableExtensionPIDController();

    heightInInches = climber.getFrontTalonPositionInInches() + Arm.ARM_HEIGHT_IN_INCHES + RobotConstants.DRIVEBASE_TO_GROUND - LEVEL_THREE_PLATFORM_HEIGHT_IN_INCHES;

    holdElbowDownWithCurrentLimit(CLIMB_CURRENT); // Brings arm down

    Logger.consoleLog("startHeightOfRobot%s, startAngleOfElbow%s ", heightInInches, elbow.getAngleInDegrees());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double deltaSetPointInInches = (heightInInches/(Math.cos(Math.toRadians(elbow.getAngleInDegrees())))) - Arm.FIXED_ARM_LENGTH_IN_INCHES;

    extension.setPIDControllerSetpointInInches(Extension.EXTENSION_OUT_IN_INCHES - deltaSetPointInInches); // Changes the setpoint
    
    if(loggingIterationCounter % 5 == 0)
      Logger.consoleLog("heightOfRobot%s, angleOfElbow%s, extensionSetpoint%s ", heightInInches, elbow.getAngleInDegrees(),Extension.EXTENSION_OUT_IN_INCHES - deltaSetPointInInches);

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // Is only set to false when CancelArmClimb command is called
    if (!userWishesToStillClimb)
      return true;

    // If arm pulls in too much and no longer makes contact with surface
    if (extension.getLengthInches() < (heightInInches / (Math.cos(Math.toRadians((elbow.getAngleInDegrees())))))) {
      return true;
    }

    return extension.isReverseLimitPressed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    releaseElbow();
    Logger.consoleLog("endHeightOfRobot%s, endAngleOfElbow%s ", heightInInches, elbow.getAngleInDegrees());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
    end();
  }

  /**
   * Holds the elbow down by setting a current limit and running the motor down at
   * full speed
   * @param currentLimit The current limit to set to (positive for forward output, negative for backwards)
   */
  private void holdElbowDownWithCurrentLimit(int currentLimit) {
    elbow.setClimbingCurrentLimit();
    //Climb current defaults to negative because of the way arm is flipped during climbing
    elbow.setCurrentControl(CLIMB_CURRENT);
  }

  /**
   * Releases the elbow by setting the current limit back to normal and setting
   * the motor power to 0
   */
  private void releaseElbow() {
    elbow.setOutput(0);
    elbow.setNormalCurrentLimit();
  }

  /**
   * Returns a boolean that keeps track of whether or not the cancel command has
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
