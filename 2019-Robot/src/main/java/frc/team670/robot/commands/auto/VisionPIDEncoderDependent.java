/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.Pose;
import frc.team670.robot.Robot;

public class VisionPIDEncoderDependent extends Command {

  private PIDController  distanceController, headingController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private static final double degreeTolerance = 0.05; //degrees
  private static final double distanceTolerance = 0.05; //inches
  private double visionHeadingControllerLowerBound = -.15, visionHeadingControllerUpperBound = .15;
  private double visionDistanceControllerLowerBound = -.7, visionDistanceControllerUpperBound = .7;

  private double distanceControllerLowerBound = 0.05, distanceControllerUpperBound = 0.05;

  private final double cameraOffset = 2.5; //distance from camera to front of the robot in inches.
  private int executeCount;
  private final double minimumAngleAdjustment = 0.03;

  private Pose robotPosition, currentPose;


  public VisionPIDEncoderDependent() {
    requires(Robot.driveBase);
    distanceController = new PIDController(P, I, D, F, new TwoEncoder_PIDSource(Robot.driveBase.getLeftEncoderCollection(), Robot.driveBase.getRightEncoderCollection()), null);
    headingController = new PIDController (P, I, D, F, Robot.visionPi.getAngleToTarget(), null);
    
    headingController.setInputRange(-30.0,  30.0);
    headingController.setOutputRange(visionHeadingControllerLowerBound, visionHeadingControllerUpperBound);
    headingController.setAbsoluteTolerance(degreeTolerance);
    headingController.setContinuous(false);

    distanceController.setOutputRange(visionDistanceControllerLowerBound, visionDistanceControllerUpperBound);
    distanceController.setAbsoluteTolerance(distanceTolerance);
    distanceController.setContinuous(false);   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
    private SensorCollection left, right;

    private int initialLeft, initialRight;

    private PIDSourceType pidSourceType; 

    public TwoEncoder_PIDSource(SensorCollection left, SensorCollection right) {
        this.left = left;
        this.right = right;
        initialLeft = left.getQuadraturePosition();
        initialRight = right.getQuadraturePosition();
        pidSourceType = PIDSourceType.kDisplacement;
    }
    
    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    @Override
    public double pidGet() {
        int displacementLeft = left.getQuadraturePosition() - initialLeft;
        int displacementRight = right.getQuadraturePosition() - initialRight;
        return (displacementLeft + displacementRight) / 2;
    }


    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSourceType = pidSource;
    }
  }


}
