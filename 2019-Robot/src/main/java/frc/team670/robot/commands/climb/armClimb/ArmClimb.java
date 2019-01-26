/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.armClimb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

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

  public static final int CLIMB_CURRENT = 10; // TODO figure required limited current

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

    heightInInches = climber.getFrontTalonPositionInInches() + RobotConstants.ARM_HEIGHT_IN_INCHES + RobotConstants.DRIVEBASE_TO_GROUND; // TODO get the actual method

    Logger.consoleLog("startHeightOfRobot%s, startAngleOfElbow%s ", heightInInches, elbow.getAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    holdElbowDownWithCurrentLimit(CLIMB_CURRENT); // Brings arm down

    double deltaSetPointInInches = (heightInInches/(Math.cos(Math.toDegrees(elbow.getAngle())))) - RobotConstants.FIXED_ARM_LENGTH_IN_INCHES;

    int deltaSetPointInTicks = MathUtils.convertExtensionInchesToTicks(deltaSetPointInInches);

    // TODO make EXTENSION_ENCODER_OUT the actual extension value of Extension at the ReadyToClimb ArmState
    extension.setPIDControllerSetpoint(Extension.EXTENSION_ENCODER_OUT - deltaSetPointInTicks); // Changes the
                                                                                                    // setpoint
    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("heightOfRobot%s, angleOfElbow%s, extensionSetpoint%s ", heightInInches, elbow.getAngle(), Extension.EXTENSION_ENCODER_OUT - deltaSetPointInTicks);

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Is only set to false when CancelArmClimb command is called
    if (!userWishesToStillClimb)
      return true;

    // If arm pulls in too much and no longer makes contact with surface
    if (extension.getLengthTicks() < (Math.cos(elbow.getAngle() / heightInInches))) {
      return true;
    }

    return extension.isReverseLimitPressed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    releaseElbow();
    Scheduler.getInstance().add(new MoveArm(Arm.getArmState(LegalState.STOW), arm));
    Logger.consoleLog("endHeightOfRobot%s, endAngleOfElbow%s ", heightInInches, elbow.getAngle());
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
   * 
   */
  private void holdElbowDownWithCurrentLimit(int currentLimit) {
    elbow.setClimbingCurrentLimit();
    //Climb current defaults to negative because of the way arm is flipped during climbing
    elbow.setCurrentControl(-CLIMB_CURRENT);
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
