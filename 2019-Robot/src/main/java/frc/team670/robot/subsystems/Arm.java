/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;
import java.util.HashMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.commands.arm.armTransitions.NeutralToCargoPickup;
import frc.team670.robot.constants.RobotConstants;

import frc.team670.robot.utils.sort.Node;
import frc.team670.robot.utils.sort.Edge;

/**
 * Stores possible arm states. Does arm-related math
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
    states.put(LegalState.START_BALL, new Neutral()); // This obviously needs to be changed
    states.put(LegalState.START_HATCH, new Neutral());
    states.put(LegalState.START_EMPTY, new Neutral()); 
    states.put(LegalState.IN_BALLGROUNDF, new Neutral()); 
    states.put(LegalState.IN_BALLSTATIONF, new Neutral());
    states.put(LegalState.IN_BALLSTATIONB, new Neutral());
    states.put(LegalState.IN_HATCHFSTATION, new Neutral()); 
    states.put(LegalState.IN_HATCHBSTATION, new Neutral()); 
    states.put(LegalState.IN_HATCHGROUNDB, new Neutral());//OOF
    states.put(LegalState.PLACE_BALLCARGOF, new Neutral());
    states.put(LegalState.PLACE_BALLCARGOB, new Neutral()); 
    states.put(LegalState.PLACE_HATCHCARGOF, new Neutral());
    states.put(LegalState.PLACE_HATCHCARGOB, new Neutral());
    states.put(LegalState.PLACE_HATCHROCKETLOWF, new Neutral()); 
    states.put(LegalState.PLACE_HATCHROCKETLOWB, new Neutral()); 
    states.put(LegalState.PLACE_HATCHROCKETMEDF, new Neutral());
    states.put(LegalState.PLACE_HATCHROCKETMEDB, new Neutral());
    /*
     *
    NEUTRAL(0), START_BALL(1), START_HATCH(2), START_EMPTY(3), IN_BALLGROUNDF(4), IN_BALLSTATIONF(5),
    IN_BALLSTATIONB(6), IN_HATCHFSTATION(7), IN_HATCHBSTATION(8), IN_HATCHGROUNDB(9),
    PLACE_BALLCARGOF(10), PLACE_BALLCARGOB(11), PLACE_HATCHCARGOF(12), PLACE_HATCHCARGOB(13),
    PLACE_HATCHROCKETLOWF(14), PLACE_HATCHROCKETLOWB(15), PLACE_HATCHROCKETMEDF(16), 
    PLACE_HATCHROCKETMEDB(17), PLACE_BALLROCKETLOWF(18), PLACE_BALLROCKETLOWB(19), 
    PLACE_BALLROCKETMEDF(20), PLACE_BALLROCKETMEDB(21), CLIMB_START(22), STOW(23), DEFENSE(24),
    IN_BALLGROUNDB(25);
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
  public static Point2D.Double getCoordPosition(double extensionLength, double wristAngle, double elbowAngle) {
    double x = extensionLength * Math.sin(elbowAngle) + RobotConstants.CLAW_LENGTH * Math.sin(wristAngle);
    double y = extensionLength * Math.cos(elbowAngle) + RobotConstants.CLAW_LENGTH * Math.cos(wristAngle) + RobotConstants.ARM_HEIGHT;
    return new Point2D.Double(x, y);
  }
 
  @Override
  public void initDefaultCommand() {
    
  }

  /**
   * Represents the different possible states of the Arm
   * //B for back, F for front
   */
  public enum LegalState {
    NEUTRAL(0), START_BALL(1), START_HATCH(2), START_EMPTY(3), IN_BALLGROUNDF(4), IN_BALLSTATIONF(5),
    IN_BALLSTATIONB(6), IN_HATCHFSTATION(7), IN_HATCHBSTATION(8), IN_HATCHGROUNDB(9),
    PLACE_BALLCARGOF(10), PLACE_BALLCARGOB(11), PLACE_HATCHCARGOF(12), PLACE_HATCHCARGOB(13),
    PLACE_HATCHROCKETLOWF(14), PLACE_HATCHROCKETLOWB(15), PLACE_HATCHROCKETMEDF(16), 
    PLACE_HATCHROCKETMEDB(17), PLACE_BALLROCKETLOWF(18), PLACE_BALLROCKETLOWB(19), 
    PLACE_BALLROCKETMEDF(20), PLACE_BALLROCKETMEDB(21), CLIMB_START(22), STOW(23), DEFENSE(24),
    IN_BALLGROUNDB(25);
    
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

    /**
     * @param extensionLength The absolute Extension length with Extension length in absolute inches with 0 being completely unextended.
     * @param elbowAngle The absolute Elbow angle with 0 being vertical in the space (180,-180) with 180 being towards the front of the robot.
     * @param wristAngle The absolute Wrist angle with 0 being in line with the arm in the space (180,-180) with 180 being towards the front of the robot.
     * @param transitions The ArmTransitions that begin at this ArmState
     */
    public ArmState(double extensionLength, double elbowAngle, double wristAngle, ArmTransition[] transitions) {
      this.extensionLength = extensionLength;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      coord = Arm.getCoordPosition(extensionLength, wristAngle, elbowAngle);
      this.transitions = transitions;
    }

    public Point2D.Double getCoordPosition() {
      return new Point2D.Double(coord.x, coord.y);
    }

    /**
     * Gets the absolute Extension length in inches.
     */
    public double getExtensionLength() {
      return extensionLength;
    }

    /**
     * Gets the absolue Elbow angle in degrees.
     */
    public double getElbowAngle() {
      return elbowAngle;
    }

    /**
     * Gets the absolute Wrist angle in degrees.
     */
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
      super(0, 45, 45, new ArmTransition[] { new NeutralToCargoPickup() });
    }
  }

}
