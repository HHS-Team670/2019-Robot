/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
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

  private ArrayList<ArmState> allowableStates;


  public Arm() {
    translationMotor = new TalonSRX(RobotMap.armTranslationMotor);
    extensionMotor = new TalonSRX(RobotMap.armExtensionMotor);
    elbowRotationMain = new TalonSRX(RobotMap.armElbowRotationMotorTalon);
    wristRotation = new TalonSRX(RobotMap.armWristRotation);

    elbowRotationSlave = new VictorSPX(RobotMap.armElbowRotationMotorVictor);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());

    allowableStates = new ArrayList<ArmState>();
    /*
     * Add in all of the states.
     */

     // This is probably not necessary, but its not like its going to take too long and it only needs to run once.
     Collections.sort(allowableStates, new Comparator<ArmState>() { // Sorts the states by their LegalState IDs
       @Override
      public int compare(ArmState lhs, ArmState rhs) {
          // -1 - less than, 1 - greater than, 0 - equal, all inversed for descending
          return lhs.getState().getId() - rhs.getState().getId();
      }
    });

  }


  /**
   * Returns the arm's point in forward facing plane relative to (0,0) at the base of the arm.
   * 
   * left the variable stuff as parameters for now
   */
  public Point2D.Double getPosition(double extensionLength, double wristAngle, double elbowAngle) {

    double x = extensionLength * Math.sin(elbowAngle) + RobotConstants.clawRadius * Math.sin(wristAngle);
    double y = extensionLength * Math.cos(elbowAngle) + RobotConstants.clawRadius * Math.cos(wristAngle) + RobotConstants.armBaseHeight;

    return new Point2D.Double(x, y);

  }

  @Override
  public void initDefaultCommand() {
    
  }

  /**
   * Represents the different possible states of the Arm
   */
  public enum LegalState {
    NEUTRAL(0), CUBE_PICKUP(1);

    private final int ID;

    LegalState(int id) {
        ID = id;
    }

    /**
     * Gets the ID of the state. This is meant to be ordered so if the state doesn't have a direct transition to a state, it transitions
     * to the state it can with the closest ID to that which should be physically closer to the desired state, and thus likely to have
     * a transition to it.
     */
    public int getId() {
        return ID;
    }
  };
  /**
   * Represents a potential state for the arm including a wrist angle, elbow angle, and extension.
   */
  public class ArmState {
    private double elbowAngle, wristAngle;
    private double extensionLength;
    private Point2D.Double coord;
    private LegalState state;

    private ArmTransition[] transitions;

    /**
     * @param transitionableStates Should h
     */
    protected ArmState(LegalState state, double extensionLength, double elbowAngle, double wristAngle, ArmTransition[] transitions) {
      this.state = state;
      this.extensionLength = extensionLength;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      coord = getPosition(extensionLength, wristAngle, elbowAngle);
      this.transitions = transitions;
    }

    public LegalState getState() {
      return state;
    }

    public Point2D.Double getCoord() {
      return new Point2D.Double(coord.x, coord.y);
    }

    public double getExtensionLength() {
      return extensionLength;
    }

    public double getElbowAngle() {
      return elbowAngle;
    }

    public double getWristAngle() {
      return wristAngle;
    }

    public CommandGroup getTransition(LegalState destination) {
      CommandGroup result = new CommandGroup();
      for(ArmTransition transition : transitions) {
        if(transition.getDestination().equals(destination)) {
          result.addSequential(transition);
          return result;
        }
      }

      /*
       * Implement this so it searches for the quickest path of transititons to the destination.
       */

      return result;

    }

  }

}
