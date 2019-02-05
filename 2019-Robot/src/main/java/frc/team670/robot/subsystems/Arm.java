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

import frc.team670.robot.Robot;
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

  public static final double ARM_HEIGHT_IN_INCHES = 5;
  public static final double CLAW_LENGTH_IN_INCHES = 8;
  public static final int FIXED_ARM_LENGTH_IN_INCHES = 0;
  public static final double OPERATOR_ARM_CONTROL_SCALAR = 0.5;

  // All of the states
  private static HashMap<LegalState, ArmState> states;
  private static ArmState currentState;

  /** Value meant only for unit testing. Do not use this anywhere else! */
  public static double unitTestExtensionDist, unitTestElbowAngle, unitTestWristAngle;

  private BaseElbow elbow;
  private BaseWrist wrist;
  private BaseExtension extension;
  private Claw claw;

  public Arm(BaseElbow elbow, BaseWrist wrist, BaseExtension extension, BaseIntake intake, Claw claw) {

    this.elbow = elbow;
    this.wrist = wrist;
    this.extension = extension;
    this.claw = claw;

    // State Setup
    states = new HashMap<LegalState, ArmState>();
    states.put(LegalState.NEUTRAL, new Neutral(this, intake));

    states.put(LegalState.START_BALL, new StartBall(this, intake));
    states.put(LegalState.START_HATCH, new StartHatch(this, intake));
    states.put(LegalState.START_EMPTY, new StartEmpty(this, intake));
  
    states.put(LegalState.GRAB_BALL_GROUND_BACK, new GrabBallGroundBack(this, intake));

    states.put(LegalState.GRAB_BALL_INTAKE, new GrabBallIntake(this, intake));

    states.put(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD, new GrabBallLoadingStationForward(this, intake));
    states.put(LegalState.READY_GRAB_BALL_LOADINGSTATION_FORWARD, new ReadyGrabBallLoadingStationForward(this, intake));
    states.put(LegalState.GRAB_BALL_LOADINGSTATION_BACK, new GrabBallLoadingStationBack(this, intake));
    states.put(LegalState.READY_GRAB_BALL_LOADINGSTATION_BACK, new ReadyGrabBallLoadingStationBack(this, intake));

    states.put(LegalState.LOW_HATCH_FORWARD, new LowHatchForward(this, intake)); 
    states.put(LegalState.READY_LOW_HATCH_FORWARD, new ReadyLowHatchForward(this, intake));
    states.put(LegalState.LOW_HATCH_BACK, new LowHatchBack(this, intake)); 
    states.put(LegalState.READY_LOW_HATCH_BACK, new ReadyLowHatchBack(this, intake));

    states.put(LegalState.PLACE_BALL_CARGOSHIP_FORWARD, new PlaceBallCargoForward(this, intake));
    states.put(LegalState.PLACE_BALL_CARGOSHIP_BACK, new PlaceBallCargoBack(this, intake));

    states.put(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, new ReadyPlaceHatchRocketMiddleForward(this, intake));
    states.put(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, new ReadyPlaceHatchRocketMiddleBack(this, intake));
    states.put(LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD, new PlaceHatchRocketMiddleForward(this, intake));
    states.put(LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK, new PlaceHatchRocketMiddleBack(this, intake));

    states.put(LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, new ReadyPlaceBallRocketLowForward(this, intake));
    states.put(LegalState.PLACE_BALL_ROCKET_LOW_FORWARD, new PlaceBallRocketLowForward(this, intake));
    states.put(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, new ReadyPlaceBallRocketLowBack(this, intake));
    states.put(LegalState.PLACE_BALL_ROCKET_LOW_BACK, new PlaceBallRocketLowBack(this, intake));

    states.put(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, new ReadyPlaceBallRocketMiddleForward(this, intake));
    states.put(LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, new PlaceBallRocketMiddleForward(this, intake));
    states.put(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, new ReadyPlaceBallRocketMiddleBack(this, intake));
    states.put(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK, new PlaceBallRocketMiddleBack(this, intake));

    states.put(LegalState.READY_TO_CLIMB, new ReadyToClimb(this, intake));
    
    states.put(LegalState.STOW, new Stow(this, intake));

    currentState = getArmState(LegalState.NEUTRAL); //Default state

    for(ArmState state : getStatesArrayList()) { // Initialize all the transitions
      state.initTransitions();
    }

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
    // for (ArmTransition transition : state.getEdges()) {
      // System.out.println("Transition Destination: " + transition.getDest().getClass().getName());
    // }
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
    double x = (extensionLength + FIXED_ARM_LENGTH_IN_INCHES) * Math.sin(elbowAngle) + CLAW_LENGTH_IN_INCHES * Math.sin(wristAngle);
    double y = (extensionLength + FIXED_ARM_LENGTH_IN_INCHES) * Math.cos(elbowAngle) + CLAW_LENGTH_IN_INCHES * Math.cos(wristAngle) + ARM_HEIGHT_IN_INCHES;
    return new Point2D.Double(x, y);
  }

  /**
   * Represents the different possible states of the Arm
   * READY = the state where the Arm is close to being able to place 
   */
  public enum LegalState {
    NEUTRAL(0), // High Neutral Position within the robot that all Transitions can run through.

    START_BALL(1), // Starting Position
    START_HATCH(2),
    START_EMPTY(3),
    
    GRAB_BALL_GROUND_BACK(4), // Ball from the ground at the back
    
    GRAB_BALL_INTAKE(5), // Ball from Intake
    
    GRAB_BALL_LOADINGSTATION_FORWARD(6), // Ball from Loading Station
    READY_GRAB_BALL_LOADINGSTATION_FORWARD(7), // Ball from Loading Station
    GRAB_BALL_LOADINGSTATION_BACK(8),
    READY_GRAB_BALL_LOADINGSTATION_BACK(9),
    
    READY_LOW_HATCH_FORWARD(10), // Low Hatch (Rocket, Cargo, and Loading Station)
    LOW_HATCH_FORWARD(11),
    READY_LOW_HATCH_BACK(12),
    LOW_HATCH_BACK(13),
    
    PLACE_BALL_CARGOSHIP_FORWARD(14), // Cargo Ship Ball
    PLACE_BALL_CARGOSHIP_BACK(15),
    
    READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD(16), // Middle Rocket Hatch
    PLACE_HATCH_ROCKET_MIDDLE_FORWARD(17),
    READY_PLACE_HATCH_ROCKET_MIDDLE_BACK(18),
    PLACE_HATCH_ROCKET_MIDDLE_BACK(19),
    
    READY_PLACE_BALL_ROCKET_LOW_FORWARD(20), // Low Ball on Rocket
    PLACE_BALL_ROCKET_LOW_FORWARD(21),
    READY_PLACE_BALL_ROCKET_LOW_BACK(22),
    PLACE_BALL_ROCKET_LOW_BACK(23),
    
    READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD(24), // Middle Ball on Rocket
    PLACE_BALL_ROCKET_MIDDLE_FORWARD(25),
    READY_PLACE_BALL_ROCKET_MIDDLE_BACK(26),
    PLACE_BALL_ROCKET_MIDDLE_BACK(27),
    
    READY_TO_CLIMB(28), // Position at full possible extension, wrist up, directly above the HAB once robot has elevated on pistons
    
    STOW(29); // Intake in, with Arm all the way down on top of it and fully retracted
  
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
    boolean isIntakeDeployed;

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
     * @param isIntakeDeployed  True if the intake is deployed (out to intake balls), false
     *                          if it is retracted into the robot.
     * @param transitions     The ArmTransitions that begin at this ArmState
     */
    protected ArmState(double elbowAngle, double wristAngle, double extensionLength, boolean isIntakeDeployed, ArmTransition[] transitions) {
      this.extensionLength = extensionLength;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      coord = Arm.getCoordPosition(elbowAngle, wristAngle, extensionLength);
      this.transitions = transitions;
      this.isIntakeDeployed = isIntakeDeployed;
    }

    public Point2D.Double getCoordPosition() {
      return new Point2D.Double(coord.x, coord.y);
    }

    /**
     * Returns the lowest point on the claw/arm which should be the place the intake is most likely to hit
     */
    public double getMaximumLowestPointOnClaw(){
      double extraClearanceInInches = 2;
      // If wrist angle is at 0, this should be the lowest point on the claw. If the
      // wrist is angled up, that does not change this calculation.
      // This should be relatively safe. It should not hit the pistons on the claw
      // either.
      double lowestPoint = getCoordPosition().getY() - Claw.MAX_CLAW_OPEN_DIAMETER / 2;
      // If the wrist is angled forward, the center point that we are using to get arm
      // coordinates will be closer to the ends of the claw so we shouldn't need to
      // add in claw diameter. The cosine function brings that addition down to 0.
      if (wrist.getAngle() > 0) {
        lowestPoint = getCoordPosition().getY() -  (Claw.MAX_CLAW_OPEN_DIAMETER / 2) * Math.abs(Math.cos(Math.toRadians(wrist.getAngle())));
      }
      //To be more safe, there is an extra buffer
      lowestPoint += extraClearanceInInches;
      return lowestPoint;
    }

    /**
     * True if the intake should be deployed at this ArmState
     */
    public boolean isIntakeDeployed() {
      return isIntakeDeployed;
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

  // NONE OF THESE CAN HAVE THE SAME COORDINATES OR EVERYTHING BREAKS IN THE A STAR SEARCH
  private class Neutral extends ArmState {
    private Neutral(Arm arm, BaseIntake intake) {
      super(0, 0, 0, false, new ArmTransition[] {
      new CommonTransition(LegalState.NEUTRAL, LegalState.START_BALL, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.START_HATCH, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.START_EMPTY, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_GROUND_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_INTAKE, arm, intake),
      // new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_LOADINGSTATION_FORWARD, arm, intake),
      // new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_LOW_HATCH_FORWARD, arm, intake), 
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_LOW_HATCH_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.PLACE_BALL_CARGOSHIP_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_TO_CLIMB, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_GRAB_BALL_LOADINGSTATION_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_GRAB_BALL_LOADINGSTATION_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.STOW, arm, intake)
      });
    }
  }

  private class StartBall extends ArmState {
    private StartBall(Arm arm, BaseIntake intake) {
      super(1, 15, 17, false, new ArmTransition[] { new CommonTransition(LegalState.START_BALL, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class StartHatch extends ArmState {
    private StartHatch(Arm arm, BaseIntake intake) {
     super(0, 3, 0, false, new ArmTransition[] { new CommonTransition(LegalState.START_HATCH, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class StartEmpty extends ArmState {
    private StartEmpty(Arm arm, BaseIntake intake) {
      super(0, 5, 0, false, new ArmTransition[] { new CommonTransition(LegalState.START_EMPTY, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class GrabBallGroundBack extends ArmState {
    private GrabBallGroundBack(Arm arm, BaseIntake intake) {
      super(0, 6, 0, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class GrabBallIntake extends ArmState {
    private GrabBallIntake(Arm arm, BaseIntake intake) {
      super(0, 7, 0, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_INTAKE, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class ReadyGrabBallLoadingStationForward extends ArmState {
    private ReadyGrabBallLoadingStationForward(Arm arm, BaseIntake intake) {
      super(0, 8, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_GRAB_BALL_LOADINGSTATION_FORWARD, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class GrabBallLoadingStationForward extends ArmState {
    private GrabBallLoadingStationForward(Arm arm, BaseIntake intake) {
      super(0, 9, 0, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class GrabBallLoadingStationBack extends ArmState {
    private GrabBallLoadingStationBack(Arm arm, BaseIntake intake) {
      super(0, 10, 0, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class ReadyGrabBallLoadingStationBack extends ArmState {
    private ReadyGrabBallLoadingStationBack(Arm arm, BaseIntake intake) {
      super(0, 11, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_GRAB_BALL_LOADINGSTATION_BACK, LegalState.NEUTRAL, arm, intake)});
    }
  }

  private class ReadyLowHatchForward extends ArmState {
    private ReadyLowHatchForward(Arm arm, BaseIntake intake) {
      super(0, 12, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_LOW_HATCH_FORWARD, LegalState.LOW_HATCH_FORWARD, arm, intake),
        new CommonTransition(LegalState.READY_LOW_HATCH_FORWARD, LegalState.NEUTRAL, arm, intake)});
    }
  }
  private class LowHatchForward extends ArmState {
    private LowHatchForward(Arm arm, BaseIntake intake) {
      super(30, 40, 6, false, new ArmTransition[] { new CommonTransition(LegalState.LOW_HATCH_FORWARD, LegalState.READY_LOW_HATCH_FORWARD, arm, intake)});
    }
  }
  private class ReadyLowHatchBack extends ArmState {
    private ReadyLowHatchBack(Arm arm, BaseIntake intake) {
      super(0, 13, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.LOW_HATCH_BACK, arm, intake),
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.NEUTRAL, arm, intake)});
    }
  }
  private class LowHatchBack extends ArmState {
    private LowHatchBack(Arm arm, BaseIntake intake) {
      super(0, 14, 0, false, new ArmTransition[] { new CommonTransition(LegalState.LOW_HATCH_BACK, LegalState.READY_LOW_HATCH_BACK, arm ,intake)});
    }
  }

  private class PlaceBallCargoForward extends ArmState{
    private PlaceBallCargoForward(Arm arm, BaseIntake intake) {
      super(0, 15, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_FORWARD, LegalState.NEUTRAL, arm ,intake)});
    }
  }

  private class PlaceBallCargoBack extends ArmState{
    private PlaceBallCargoBack(Arm arm, BaseIntake intake) {
      super(0, 16, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.NEUTRAL, arm ,intake)});
    }
  }

  private class ReadyPlaceHatchRocketMiddleForward extends ArmState{
    private ReadyPlaceHatchRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(0, 17, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD, arm ,intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, LegalState.NEUTRAL, arm ,intake)});
    }
  }

  private class PlaceHatchRocketMiddleForward extends ArmState{
    private PlaceHatchRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(0, 18, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, arm ,intake)});
    }
  }

  private class ReadyPlaceHatchRocketMiddleBack extends ArmState{
    private ReadyPlaceHatchRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(0, 19, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK, arm ,intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.NEUTRAL, arm ,intake)});
    }
  }

  private class PlaceHatchRocketMiddleBack extends ArmState{
    private PlaceHatchRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(0, 20, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm ,intake)});
    }
  }

  private class ReadyPlaceBallRocketLowForward extends ArmState{
    private ReadyPlaceBallRocketLowForward(Arm arm, BaseIntake intake) {
      super(0, 21, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, LegalState.PLACE_BALL_ROCKET_LOW_FORWARD, arm ,intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.NEUTRAL, arm ,intake)});
    }
  }

  private class PlaceBallRocketLowForward extends ArmState{
    private PlaceBallRocketLowForward(Arm arm, BaseIntake intake) {
      super(0, 22, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_LOW_FORWARD, LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, arm ,intake)});
    }
  }

  private class ReadyPlaceBallRocketLowBack extends ArmState {
    private ReadyPlaceBallRocketLowBack(Arm arm, BaseIntake intake) {
      super(0, 23, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.NEUTRAL, arm, intake) });
    }
  }

  private class PlaceBallRocketLowBack extends ArmState {
    private PlaceBallRocketLowBack(Arm arm, BaseIntake intake) {
      super(0, 24, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_LOW_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake)});
    }
  }
  
  private class ReadyPlaceBallRocketMiddleForward extends ArmState {
    private ReadyPlaceBallRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(0, 25, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, LegalState.NEUTRAL, arm, intake) });
    }
  }

  private class PlaceBallRocketMiddleForward extends ArmState {
    private PlaceBallRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(0, 26, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, arm, intake)});
    }
  }

  private class ReadyPlaceBallRocketMiddleBack extends ArmState {
    private ReadyPlaceBallRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(0, 27, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.NEUTRAL, arm, intake) });
    }
  }

  private class PlaceBallRocketMiddleBack extends ArmState {
    private PlaceBallRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(0, 28, 0, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake)});
    }
  }

  private class ReadyToClimb extends ArmState {
    private ReadyToClimb(Arm arm, BaseIntake intake) {
      super(0, 29, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_TO_CLIMB, LegalState.NEUTRAL, arm, intake)});
    }
  }
  private class Stow extends ArmState {
      private Stow(Arm arm, BaseIntake intake) {
        super(0, 30, 0, false, new ArmTransition[] { new CommonTransition(LegalState.STOW, LegalState.NEUTRAL, arm, intake)});
      } 
  }
}
