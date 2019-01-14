/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.dataCollection.Pose;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

public class VisionPIDEncoderDependent extends Command {

  private Pose pose;

  private PIDController  distanceController, headingController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private static final double degreeTolerance = 0.05; //degrees
  private static final double distanceTolerance = 0.05; //inches
  private double visionHeadingControllerLowerOutput = -.15, visionHeadingControllerUpperOutput = .15;
  private double visionDistanceControllerLowerBound = -.7, visionDistanceControllerUpperBound = .7;

  private double distanceControllerLowerBound = 0.05, distanceControllerUpperBound = 0.05;

  private final double cameraOffset = 2.5; //distance from camera to front of the robot in inches.
  private int executeCount;
  private final double minimumAngleAdjustment = 0.03;

  private Pose robotPosition, currentPose;


  public VisionPIDEncoderDependent() {
    requires(Robot.driveBase);
    distanceController = new PIDController(P, I, D, F, new TwoEncoder_PIDSource(Robot.driveBase.getLeftEncoder(), Robot.driveBase.getRightEncoder()), null);
    headingController = new PIDController (P, I, D, F, Robot.visionPi.getAngleToWallTarget(), null);
    
    headingController.setInputRange(-180.0,  180.0);
    headingController.setOutputRange(visionHeadingControllerLowerOutput, visionHeadingControllerUpperOutput);
    headingController.setAbsoluteTolerance(degreeTolerance);
    headingController.setContinuous(true);

    distanceController.setOutputRange(visionDistanceControllerLowerBound, visionDistanceControllerUpperBound);
    distanceController.setAbsoluteTolerance(distanceTolerance);
    distanceController.setContinuous(false);   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Pose.resetPoseInputs();
    robotPosition = new Pose();
    Logger.consoleLog("Initialized VisionPIDEncoderDependent");

    headingController.setSetpoint(0);
    distanceController.setSetpoint(0 + cameraOffset);

    executeCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    currentPose = new Pose();

    /** changed output range to insure that the distanceController isn't going into a negative range */
    double distanceOutput = distanceController.get() * -1;
    double headingOutput = headingController.get();

    if(headingOutput >=0) {
      headingOutput += minimumAngleAdjustment;
    } else {
      headingOutput -=  minimumAngleAdjustment; 
    }

    double leftSpeed = distanceOutput - headingOutput;
    double rightSpeed = distanceOutput + headingOutput;

    Robot.driveBase.tankDrive(leftSpeed, rightSpeed);
    if (executeCount % 5 == 0) {
      Logger.consoleLog("Executing VisionTargetPidDrive: headingOutput:%s, distanceOutput:%s, leftSpeed:%s, rightSpeed:%s", headingOutput, distanceOutput, leftSpeed, rightSpeed);
    }

    robotPosition.update(Robot.driveBase.getLeftEncoderPosition(), Robot.driveBase.getRightEncoderPosition(), Robot.sensors.getYawDouble());



    executeCount ++;

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

  private class TwoEncoder_PIDSource implements PIDSource {
    private CANEncoder left, right;

    private double initialLeft, initialRight;

    private PIDSourceType pidSourceType; 

    public TwoEncoder_PIDSource(CANEncoder left, CANEncoder right) {
        this.left = left;
        this.right = right;
        initialLeft = left.getPosition();
        initialRight = right.getPosition();
        pidSourceType = PIDSourceType.kDisplacement;
    }
    
    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    @Override
    public double pidGet() {
        double displacementLeft = left.getPosition() - initialLeft;
        double displacementRight = right.getPosition() - initialRight;
        return MathUtils.convertDriveBaseTicksToInches((displacementLeft + displacementRight) / 2);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSourceType = pidSource;
    }
  }

}
