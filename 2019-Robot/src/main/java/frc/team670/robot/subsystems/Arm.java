/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;

import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.commands.arm.armTransitions.CommonTransition;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.utils.sort.Node;

/**
 * Stores possible arm states. Does arm-related math Represents the arm
 * mechanism on the robot. Link to a model of the arm: https://a360.co/2TLH2NO
 * 
 * @author shaylandias, ctchen, rghosh670
 */
public class Arm {

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
    // states.put(LegalState.INTAKE_BALL_INTAKE_FORWARD, new IntakeBallIntakeForward(this));
    // states.put(LegalState.READY_TO_CLIMB, new ReadyToClimb(this));
    // states.put(LegalState.START_BALL, new Neutral(this)); // This obviously needs
    // to be changed
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
    states.put(LegalState.PLACE_HATCH_ROCKET_LOW_FORWARD, new LowHatchPlace(this)); 
    // states.put(LegalState.PLACE_HATCHROCKETLOWB, new Neutral(this)); 
    // states.put(LegalState.PLACE_HATCHROCKETMEDF, new Neutral(this));
    // states.put(LegalState.PLACE_HATCHROCKETMEDB, new Neutral(this));
    currentState = getArmState(LegalState.NEUTRAL); //Default state

    for(ArmState state : getStatesArrayList()) { // Initialize all the transitions
      state.initTransitions();
    }

    /*
     *
     * NEUTRAL(0), START_BALL(1), START_HATCH(2), START_EMPTY(3), IN_BALLGROUNDF(4),
     * IN_BALLSTATIONF(5), IN_BALLSTATIONB(6), IN_HATCHFSTATION(7),
     * IN_HATCHBSTATION(8), IN_HATCHGROUNDB(9), PLACE_BALLCARGOF(10),
     * PLACE_BALLCARGOB(11), PLACE_HATCHCARGOF(12), PLACE_HATCHCARGOB(13),
     * PLACE_HATCHROCKETLOWF(14), PLACE_HATCHROCKETLOWB(15),
     * PLACE_HATCHROCKETMEDF(16), PLACE_HATCHROCKETMEDB(17),
     * PLACE_BALLROCKETLOWF(18), PLACE_BALLROCKETLOWB(19), PLACE_BALLROCKETMEDF(20),
     * PLACE_BALLROCKETMEDB(21), CLIMB_START(22), STOW(23), DEFENSE(24),
     * IN_BALLGROUNDB(25);
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

  public static ArrayList<ArmState> getStatesArrayList() {
    return new ArrayList<ArmState>(states.values());
  }

  /**
   * Sets the current state of the arm.
   */
  public static void setState(ArmState state) {
    for (ArmTransition transition : state.getEdges()) {
      System.out.println("Transition Destination: " + transition.getDest().getClass().getName());
    }
    currentState = state;
  }

  /**
   * Gets the State that the arm most recently was located at.
   */
  public static ArmState getCurrentState() {
    return currentState;
  }

  /**
   * Gets the ArmState object that corresponds to the LegalState Ex. If you want
   * the Neutral ArmState, use 'getArmState(LegalState.NEUTRAL)''
   */
  public static ArmState getArmState(LegalState state) {
    return states.get(state);
  }

  /**
   * Returns the arm's point in forward facing plane relative to (0,0) at the base
   * of the arm.
   */
  public static Point2D.Double getCoordPosition(double elbowAngle, double wristAngle, double extensionLength) {
    double x = extensionLength * Math.sin(elbowAngle) + RobotConstants.CLAW_RADIUS_IN_INCHES * Math.sin(wristAngle);
    double y = extensionLength * Math.cos(elbowAngle) + RobotConstants.CLAW_RADIUS_IN_INCHES * Math.cos(wristAngle)
        + RobotConstants.ARM_HEIGHT_IN_INCHES;
    return new Point2D.Double(x, y);
  }

  /**
   * Represents the different possible states of the Arm //B for back, F for front
   */
  public enum LegalState {
    // ACTION_GAMEPIECE_LOCATION(PLACE AT OR INTAKE FROM)_FORWARD/BACK
    NEUTRAL(0), START_BALL(1), START_HATCH(2), START_EMPTY(3), INTAKE_BALL_GROUND_FORWARD(4), INTAKE_BALL_GROUND_BACK(5),
    INTAKE_BALL_INTAKE_FORWARD(6), INTAKE_BALL_LOADINGSTATION_FORWARD(7), INTAKE_BALL_LOADINGSTATION_BACK(8),
    INTAKE_HATCH_LOADINGSTATION_FORWARD(9), INTAKE_HATCH_LOADINGSTATION_BACK(10), INTAKE_HATCH_GROUND_BACK(11),
    PLACE_BALL_CARGOSHIP_FORWARD(12), PLACE_BALL_CARGOSHIP_BACK(13), PLACE_HATCH_CARGOSHIP_F(14),
    PLACE_HATCH_CARGOSHIP_BACK(15), PLACE_HATCH_ROCKET_LOW_FORWARD(16), PLACE_HATCH_ROCKET_LOW_BACK(17),
    PLACE_HATCH_ROCKET_MIDDLE_FORWARD(18), PLACE_HATCH_ROCKET_MIDDLE_BACK(19), PLACE_BALL_ROCKET_LOW_FORWARD(20),
    PLACE_BALL_ROCKET_LOW_BACK(21), PLACE_BALL_ROCKET_MIDDLE_FORWARD(22), PLACE_BALL_ROCKET_MIDDLE_BACK(23),
    READY_TO_CLIMB(24), STOW(25), DEFENSE(26);
    // STOW means that the intake is in and the arm is on top of the intake. Probably the same configuration as DEFENSE

    private final int ID;

    LegalState(int id) {
      ID = id;
    }

    /**
     * Gets the ID of the state. This is meant to be ordered so if the state doesn't
     * have a direct transition to a state, it transitions to the state it can with
     * the closest ID to that which should be physically closer to the desired
     * state, and thus likely to have a transition to it.
     */
    public int getId() {
      return ID;
    }
  };

  /**
   * Represents a potential state for the arm including a wrist angle, elbow
   * angle, and extension.
   */
  public class ArmState implements Node {
    private double elbowAngle, wristAngle;
    private double extensionLength;
    private Point2D.Double coord;

    private ArmTransition[] transitions;

    /**
     * @param extensionLength The absolute Extension length with Extension length in
     *                        absolute inches with 0 being completely unextended.
     * @param elbowAngle      The absolute Elbow angle with 0 being vertical in the
     *                        space (180,-180) with 180 being towards the front of
     *                        the robot.
     * @param wristAngle      The absolute Wrist angle with 0 being in line with the
     *                        arm in the space (180,-180) with 180 being towards the
     *                        front of the robot.
     * @param transitions     The ArmTransitions that begin at this ArmState
     */
    protected ArmState(double elbowAngle, double wristAngle, double extensionLength, ArmTransition[] transitions) {
      this.extensionLength = extensionLength;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      coord = Arm.getCoordPosition(elbowAngle, wristAngle, extensionLength);
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

    public void initTransitions() {
      for (ArmTransition transition : transitions) {
        transition.initTransition();
      }
    }

    @Override
    public ArmTransition[] getEdges() {
      return transitions;
    }

    @Override
    public int getHeuristicDistance(Node other) {
      ArmState state2 = (ArmState) other;
      return (int) (Math.sqrt((this.coord.getX() - state2.coord.getX()) * (this.coord.getX() - state2.coord.getX())
          + (this.coord.getY() - state2.coord.getY()) * (this.coord.getY() - state2.coord.getY())));
    }
  }

  private class Neutral extends ArmState {
    private Neutral(Arm arm) {
      super(0, 0, 0, new ArmTransition[] { new CommonTransition(LegalState.NEUTRAL, LegalState.PLACE_HATCH_ROCKET_LOW_FORWARD, arm) });
    }
  }

  private class LowHatchPlace extends ArmState {
    private LowHatchPlace(Arm arm) {
      super(30, 40, 6, new ArmTransition[] { new CommonTransition(LegalState.PLACE_HATCH_ROCKET_LOW_FORWARD, LegalState.NEUTRAL, arm)});
    }
  }

}
