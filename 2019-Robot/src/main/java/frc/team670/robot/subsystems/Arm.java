/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the arm mechanism on the robot.
 * Link to a model of the arm: https://a360.co/2TLH2NO
 * @author shaylandias
 */
public class Arm extends Subsystem {
  
  private TalonSRX translationMotor;
  private TalonSRX extensionMotor;
  private TalonSRX elbowRotationMain;
  private VictorSPX elbowRotationSlave;
  private TalonSRX wristRotation;

  private ArmState[] allowableStates;


  public Arm() {
    translationMotor = new TalonSRX(RobotMap.armTranslationMotor);
    extensionMotor = new TalonSRX(RobotMap.armExtensionMotor);
    elbowRotationMain = new TalonSRX(RobotMap.armElbowRotationMotorTalon);
    wristRotation = new TalonSRX(RobotMap.armWristRotation);

    elbowRotationSlave = new VictorSPX(RobotMap.armElbowRotationMotorVictor);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());

  }


  /**
   * Returns the arm's point in forward facing plane relative to (0,0) at the base of the arm.
   * 
   * left the variable stuff as parameters for now
   */
  public Point2D.Double getPosition(double armLength, double wristAngle, double elbowAngle) {

    double x = armLength * Math.sin(elbowAngle) + RobotConstants.clawRadius * Math.sin(wristAngle);
    double y = armLength * Math.cos(elbowAngle) + RobotConstants.clawRadius * Math.cos(wristAngle) + RobotConstants.armBaseHeight;

    return new Point2D.Double(x, y);

  }

  @Override
  public void initDefaultCommand() {
    
  }

  public class ArmState {
    private double elbowAngle, wristAngle;
    private Point2D.Double coord;
  }

  public class ArmTransition {
    private Point2D.Double startCoord, endCoord;
  }

}
