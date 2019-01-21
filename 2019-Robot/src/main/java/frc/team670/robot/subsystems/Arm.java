/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.commands.arm.armTransitions.NeutralToCargoPickup;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.Sort.Node;

/**
 * Represents the arm mechanism on the robot. Link to a model of the arm:
 * https://a360.co/2TLH2NO
 * TODO: we won't be using legalstates, we can get rid of them
 * @author shaylandias, ctchen, rghosh670
 */
public class Arm extends Subsystem {

  // All of the states
  private static HashMap<LegalState, ArmState> states = new HashMap<LegalState, ArmState>();
  private static ArmState currentState;

  public Arm() {
    // State Setup
    currentState = new Neutral(); //Default state
    states = new HashMap<LegalState, ArmState>();
    states.put(LegalState.NEUTRAL, new Neutral());
    states.put(LegalState.CARGO_PICKUP, new Neutral()); // This obviously needs to be changed
    /*
     * Add in all of the states here.
     */

  }

  /**
   * Sets the current state of the arm.
   */
  public static void setState(ArmState state) {
    currentState = state;
  }

  /**
   * Gets the State that the arm most recently was located at.
   */
  public static ArmState getCurrentState(){
    return currentState;
  }

  /**
   * Gets the ArmState object that corresponds to the LegalState
   * Ex. If you want the Neutral ArmState, use 'getArmState(LegalState.NEUTRAL)''
   */
  public static ArmState getArmState(LegalState state) {
    return states.get(state);
  }

  /**
   * Returns the arm's point in forward facing plane relative to (0,0) at the base of the arm.
   * 
   * left the variable stuff as parameters for now
   */
  public Point2D.Double getPosition(double extensionLength, double wristAngle, double elbowAngle) {
    double x = extensionLength * Math.sin(elbowAngle) + RobotConstants.CLAW_RADIUS * Math.sin(wristAngle);
    double y = extensionLength * Math.cos(elbowAngle) + RobotConstants.CLAW_RADIUS * Math.cos(wristAngle) + RobotConstants.ARM_START_HEIGHT;
    return new Point2D.Double(x, y);
  }
 
  @Override
  public void initDefaultCommand() {
    
  }

  /**
   * Represents the different possible states of the Arm
   */
  public enum LegalState {
    NEUTRAL(0), CARGO_PICKUP(1);

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
  public class ArmState implements Node {
    private double elbowAngle, wristAngle;
    private double extensionLength;
    private Point2D.Double coord;

    private ArmTransition[] transitions;

    public ArmState(double extensionLength, double elbowAngle, double wristAngle, ArmTransition[] transitions) {
      this.extensionLength = extensionLength;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      coord = getPosition(extensionLength, wristAngle, elbowAngle);
      this.transitions = transitions;
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

    @Override
    public ArmTransition[] getEdges() {
      return transitions;
    }

    @Override
    public int getHeuristicDistance(Node other){
      ArmState state2 = (ArmState)other;
      return (int)(Math.sqrt((this.coord.getX()-state2.coord.getX())*(this.coord.getX()-state2.coord.getX())+(this.coord.getY()-state2.coord.getY())*(this.coord.getY()-state2.coord.getY())));
    }
  }

  private class Neutral extends ArmState {
    public Neutral() {
      super(0, 45, 45, new ArmTransition[] {new NeutralToCargoPickup()});
    }
  }

}
