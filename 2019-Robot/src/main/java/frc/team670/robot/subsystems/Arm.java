/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;
import java.util.HashMap;

import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.commands.arm.armTransitions.NeutralToLowerHatch;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.utils.sort.Node;

/**
 * Stores possible arm states. Does arm-related math
 * Represents the arm mechanism on the robot. Link to a model of the arm:
 * https://a360.co/2TLH2NO
 * @author shaylandias, ctchen, rghosh670
 */
public class Arm  {

  // All of the states
  private static HashMap<LegalState, ArmState> states;
  private static ArmState currentState;

  /** Value meant only for unit testing. Do not use this anywhere else! */
  public static double unitTestExtensionDist, unitTestElbowAngle, unitTestWristAngle;

  private BaseElbow elbow;
  private BaseWrist wrist;
  private BaseExtension extension;


  public Arm(BaseElbow elbow, BaseWrist wrist, BaseExtension extension) {

    this.elbow = elbow;
    this.wrist = wrist;
    this.extension = extension;

    // State Setup
    states = new HashMap<LegalState, ArmState>();
    states.put(LegalState.NEUTRAL, new Neutral(this));
    // states.put(LegalState.START_BALL, new Neutral(this)); // This obviously needs to be changed
    // states.put(LegalState.START_HATCH, new Neutral(this));
    // states.put(LegalState.START_EMPTY, new Neutral(this)); 
    // states.put(LegalState.IN_BALLGROUNDF, new Neutral(this)); 
    // states.put(LegalState.IN_BALLSTATIONF, new Neutral(this));
    // states.put(LegalState.IN_BALLSTATIONB, new Neutral(this));
    // states.put(LegalState.IN_HATCHFSTATION, new Neutral(this)); 
    // states.put(LegalState.IN_HATCHBSTATION, new Neutral(this)); 
    // states.put(LegalState.IN_HATCHGROUNDB, new Neutral(this));//OOF
    // states.put(LegalState.PLACE_BALLCARGOF, new Neutral(this));
    // states.put(LegalState.PLACE_BALLCARGOB, new Neutral(this)); 
    // states.put(LegalState.PLACE_HATCHCARGOF, new Neutral(this));
    // states.put(LegalState.PLACE_HATCHCARGOB, new Neutral(this));
    // states.put(LegalState.PLACE_HATCHROCKETLOWF, new Neutral(this)); 
    // states.put(LegalState.PLACE_HATCHROCKETLOWB, new Neutral(this)); 
    // states.put(LegalState.PLACE_HATCHROCKETMEDF, new Neutral(this));
    // states.put(LegalState.PLACE_HATCHROCKETMEDB, new Neutral(this));
    currentState = new Neutral(this); //Default state
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

  public BaseElbow getElbow() {
    return elbow;
  }

  public BaseWrist getWrist() {
    return wrist;
  }

  public BaseExtension getExtension() {
    return extension;
  }

  public static HashMap<LegalState, ArmState> getStates() {
    return states;
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
   */
  public static Point2D.Double getCoordPosition(double extensionLength, double wristAngle, double elbowAngle) {
    double x = extensionLength * Math.sin(elbowAngle) + RobotConstants.CLAW_RADIUS_IN_INCHES * Math.sin(wristAngle);
    double y = extensionLength * Math.cos(elbowAngle) + RobotConstants.CLAW_RADIUS_IN_INCHES * Math.cos(wristAngle) + RobotConstants.ARM_HEIGHT_IN_INCHES;
    return new Point2D.Double(x, y);
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
    protected ArmState(double extensionLength, double elbowAngle, double wristAngle, ArmTransition[] transitions) {
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
    private Neutral(Arm arm) {
      super(0, 45, 45, new ArmTransition[] { new NeutralToLowerHatch(arm) });
    }
  }

}
