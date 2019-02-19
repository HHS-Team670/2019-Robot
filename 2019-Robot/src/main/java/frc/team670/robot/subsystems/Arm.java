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

import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.commands.arm.armTransitions.CommonTransition;
import frc.team670.robot.commands.arm.armTransitions.GrabBallIntakeToNeutral;
import frc.team670.robot.commands.arm.armTransitions.NeutralToGrabBallIntake;
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

  public enum HeldItem {NONE, BALL, HATCH};
  private HeldItem heldItem;

  public static final double ARM_HEIGHT_IN_INCHES = 5;

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

    heldItem = HeldItem.NONE; // TODO set this up to reflect starting state

    // State Setup
    states = new HashMap<LegalState, ArmState>();
    states.put(LegalState.NEUTRAL, new Neutral(this, intake));

    states.put(LegalState.START, new Start(this, intake));
  
    states.put(LegalState.GRAB_HATCH_GROUND_BACK, new GrabHatchGroundBack(this, intake));
    states.put(LegalState.READY_GRAB_HATCH_GROUND_BACK, new ReadyGrabHatchGroundBack(this, intake));

    states.put(LegalState.GRAB_BALL_GROUND_BACK, new GrabBallGroundBack(this, intake));

    states.put(LegalState.GRAB_BALL_INTAKE, new GrabBallIntake(this, intake));

    states.put(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD, new GrabBallLoadingStationForward(this, intake));
    // states.put(LegalState.READY_GRAB_BALL_LOADINGSTATION_FORWARD, new ReadyGrabBallLoadingStationForward(this, intake));
    states.put(LegalState.GRAB_BALL_LOADINGSTATION_BACK, new GrabBallLoadingStationBack(this, intake));
    // states.put(LegalState.READY_GRAB_BALL_LOADINGSTATION_BACK, new ReadyGrabBallLoadingStationBack(this, intake));

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

    // states.put(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, new ReadyPlaceBallRocketMiddleForward(this, intake));
    states.put(LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, new PlaceBallRocketMiddleForward(this, intake));
    states.put(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, new ReadyPlaceBallRocketMiddleBack(this, intake));
    states.put(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK, new PlaceBallRocketMiddleBack(this, intake));

    states.put(LegalState.READY_TO_CLIMB, new ReadyToClimb(this, intake));
    
    states.put(LegalState.STOW, new Stow(this, intake));

    currentState = getArmState(LegalState.START); //Default state

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
    double x = (extensionLength + Extension.FIXED_LENGTH) * Math.sin(elbowAngle) + Claw.LENGTH_IN_INCHES * Math.sin(wristAngle + elbowAngle);
    double y = (extensionLength + Extension.FIXED_LENGTH) * (elbowAngle < 0 ? -1 : 1) * Math.cos(elbowAngle) + Claw.LENGTH_IN_INCHES * (wristAngle + elbowAngle < 0 ? -1 : 1) * Math.cos(wristAngle + elbowAngle) + ARM_HEIGHT_IN_INCHES;
    return new Point2D.Double(x, y);
  }

  /**
   * Returns the arm's point in forward facing plane relative to (0,0) at the base
   * of the arm.
   */
  public static double getCurrentLowestPointOnArm(double elbowAngle, double wristAngle, double extensionLength) {
    double y = getCoordPosition(elbowAngle, wristAngle, extensionLength).getY();
    double extraClearanceInInches = 2;
      // If wrist angle is at 0, this should be the lowest point on the claw. If the
      // wrist is angled up, that does not change this calculation.
      // This should be relatively safe. It should not hit the pistons on the claw
      // either.
      double lowestPoint = y - Claw.MAX_CLAW_OPEN_DIAMETER / 2;
      // If the wrist is angled forward, the center point that we are using to get arm
      // coordinates will be closer to the ends of the claw so we shouldn't need to
      // add in claw diameter. The cosine function brings that addition down to 0.
      if (wristAngle > 0) {
        lowestPoint = y -  (Claw.MAX_CLAW_OPEN_DIAMETER / 2) * Math.abs(Math.cos(Math.toRadians(wristAngle)));
      }
      //To be more safe, there is an extra buffer
      lowestPoint += extraClearanceInInches;
      return lowestPoint;
  }

  public void setHeldItem(HeldItem item) {
    heldItem = item;
  }

  public HeldItem getHeldItem() {
    return heldItem;
  }

  /**
   * Represents the different possible states of the Arm
   * READY = the state where the Arm is close to being able to place 
   */
  public enum LegalState {
    NEUTRAL(0), // High Neutral Position within the robot that all Transitions can run through.

    START(1), // Starting Position
    
    READY_GRAB_HATCH_GROUND_BACK(30), // Hatch from the ground at the back
    GRAB_HATCH_GROUND_BACK(31),

    GRAB_BALL_GROUND_BACK(4), // Ball from the ground at the back
    
    GRAB_BALL_INTAKE(5), // Ball from Intake
    
    GRAB_BALL_LOADINGSTATION_FORWARD(6), // Ball from Loading Station
    // READY_GRAB_BALL_LOADINGSTATION_FORWARD(7), // Ball from Loading Station
    GRAB_BALL_LOADINGSTATION_BACK(8),
    // READY_GRAB_BALL_LOADINGSTATION_BACK(9),
    
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
    
    // READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD(24), // Middle Ball on Rocket
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
     * @param elbowAngle      The absolute Elbow angle with 0 being vertical in the
     *                        space (180,-180) with 180 being towards the front of
     *                        the robot.
     * @param wristAngle      The absolute Wrist angle with 0 being in line with the
     *                        arm in the space (180,-180) with 180 being towards the
     *                        front of the robot.
     * @param extensionLength The absolute Extension length with Extension length in
     *                        absolute inches with 0 being completely unextended.
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
      double extraClearanceInInches = 1.5;
      // If wrist angle is at 0, this should be the lowest point on the claw. If the
      // wrist is angled up, that does not change this calculation.
      // This should be relatively safe. It should not hit the pistons on the claw
      // either.
      double lowestPoint = getCoordPosition().getY() - Claw.MAX_CLAW_OPEN_DIAMETER / 2;
      // If the wrist is angled forward, the center point that we are using to get arm
      // coordinates will be closer to the ends of the claw so we shouldn't need to
      // add in claw diameter. The cosine function brings that addition down to 0.
      if (wrist.getAngleInDegrees() > 0) {
        lowestPoint = getCoordPosition().getY() -  (Claw.MAX_CLAW_OPEN_DIAMETER / 2) * Math.abs(Math.cos(Math.toRadians(wrist.getAngleInDegrees())));
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
     * Gets the absolute Elbow angle in degrees.
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

    @Override
    public int compareTo(Node o) {
      return (this == o) ? 0 : 1;
    }
  }

  public class PlaceGrabState extends ArmState {
    private boolean isFront;
    private boolean isLowTarget;
    private double distanceFromTarget;

    public PlaceGrabState(double elbowAngle, double wristAngle, double extensionLength, boolean isIntakeDeployed, ArmTransition[] transitions, boolean isFront, double distanceFromTarget, boolean isLowTarget) {
      super(elbowAngle,  wristAngle,  extensionLength, isIntakeDeployed, transitions);
      this.isFront = isFront;
      this.distanceFromTarget = distanceFromTarget;
    }

    public boolean getIsFront(){
      return isFront;
    }

    public double getDistanceFromTarget(){
      return distanceFromTarget;
    }

    public boolean getIsLowTarget(){
      return isLowTarget;
    }
  }

  // NONE OF THESE CAN HAVE THE SAME COORDINATES OR EVERYTHING BREAKS IN THE A STAR SEARCH
  private class Neutral extends ArmState {
    private Neutral(Arm arm, BaseIntake intake) {
      super(50.067, -50.067, 0, false, new ArmTransition[] {
      // new CommonTransition(LegalState.NEUTRAL, LegalState.START, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_GROUND_BACK, arm, intake),
      new NeutralToGrabBallIntake(arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_LOW_HATCH_FORWARD, arm, intake), 
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_LOW_HATCH_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.PLACE_BALL_CARGOSHIP_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.READY_TO_CLIMB, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.GRAB_BALL_LOADINGSTATION_FORWARD, arm, intake),
      new CommonTransition(LegalState.NEUTRAL, LegalState.STOW, arm, intake)
      });
    }
  }

  private class Start extends ArmState {
    private Start(Arm arm, BaseIntake intake) {
      super(80.6, 99, 0, false, new ArmTransition[] { new CommonTransition(LegalState.START, LegalState.NEUTRAL, arm, intake)});
    }
  }

  //Transitions commented out because uneccesary sequence of movement
  private class ReadyGrabHatchGroundBack extends ArmState {
    private ReadyGrabHatchGroundBack(Arm arm, BaseIntake intake) {
      super(-126, -54, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.NEUTRAL, arm, intake), 
        new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_GRAB_HATCH_GROUND_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake) 
      });
    }
  }

  private class GrabHatchGroundBack extends ArmState {
    private GrabHatchGroundBack(Arm arm, BaseIntake intake) {
      super(-130.8, -49.2, 1.2488, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_HATCH_GROUND_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake)});
    }
  }

  private class GrabBallGroundBack extends ArmState {
    private GrabBallGroundBack(Arm arm, BaseIntake intake) {
      super(-119.492, -60.508, 0, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.NEUTRAL, arm, intake) , 
        new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), 
        new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        // new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.GRAB_BALL_GROUND_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake)
      });
    }
  }

  private class GrabBallIntake extends ArmState {
    private GrabBallIntake(Arm arm, BaseIntake intake) {
      super(87.5, 84, 6.56, true, new ArmTransition[] { new GrabBallIntakeToNeutral(arm, intake)});
    }
  }

  // Changed to neutral because ready is not needed for this state
  private class GrabBallLoadingStationForward extends PlaceGrabState {
    private GrabBallLoadingStationForward(Arm arm, BaseIntake intake) {
      super(78, -45, 7, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD, LegalState.NEUTRAL, arm, intake)}, true, 4.0, true);
    }
  }

  // Changed to neutral because ready is not needed for this state
  private class GrabBallLoadingStationBack extends PlaceGrabState {
    private GrabBallLoadingStationBack(Arm arm, BaseIntake intake) {
      super(-72, 35, 0, false, new ArmTransition[] { new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.NEUTRAL, arm, intake), 
      new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        // new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.GRAB_BALL_GROUND_BACK, arm, intake), // Probably not needed 
        // new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), // Probably not needed
        new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        // new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), // Probably not needed
        new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.GRAB_BALL_LOADINGSTATION_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake) 
    }, false, 36.765, true);
    }
  }

  private class ReadyLowHatchForward extends ArmState {
    private ReadyLowHatchForward(Arm arm, BaseIntake intake) {
      super(103, -17, 3.2, false, new ArmTransition[] { new CommonTransition(LegalState.READY_LOW_HATCH_FORWARD, LegalState.LOW_HATCH_FORWARD, arm, intake), 
        new CommonTransition(LegalState.READY_LOW_HATCH_FORWARD, LegalState.NEUTRAL, arm, intake)
      });
    }
  }
  private class LowHatchForward extends PlaceGrabState {
    private LowHatchForward(Arm arm, BaseIntake intake) {
      super(103, -17, 9.7, false, new ArmTransition[] { new CommonTransition(LegalState.LOW_HATCH_FORWARD, LegalState.READY_LOW_HATCH_FORWARD, arm, intake)}, true, 4.0, true);
    }
  } 
  private class ReadyLowHatchBack extends ArmState {
    private ReadyLowHatchBack(Arm arm, BaseIntake intake) {
      super(-105.935, 15.935, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.LOW_HATCH_BACK, arm, intake),
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.NEUTRAL, arm, intake),
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.GRAB_BALL_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake), 
        // new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_LOW_HATCH_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake) 
      });
    }
  }
  private class LowHatchBack extends PlaceGrabState {
    private LowHatchBack(Arm arm, BaseIntake intake) {
      super(-106, 15, 0.03, false, new ArmTransition[] { new CommonTransition(LegalState.LOW_HATCH_BACK, LegalState.READY_LOW_HATCH_BACK, arm ,intake)}, false, 10.731, true);
    }
  }

  //Bumper can go under the cargoship
  private class PlaceBallCargoForward extends PlaceGrabState{
    private PlaceBallCargoForward(Arm arm, BaseIntake intake) {
      super(56, 38, 11.7, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_FORWARD, LegalState.NEUTRAL, arm ,intake)}, true, 4.0, true);
    }
  }

  private class PlaceBallCargoBack extends PlaceGrabState{
    private PlaceBallCargoBack(Arm arm, BaseIntake intake) {
      super(-45.5, -40, 3.8, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.NEUTRAL, arm ,intake) ,
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.GRAB_BALL_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake), 
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), 
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.PLACE_BALL_CARGOSHIP_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake) 
      }, false, 24.472, true);
    }
  }

  private class ReadyPlaceHatchRocketMiddleForward extends ArmState{
    private ReadyPlaceHatchRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(37, 49, 8.5, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD, arm ,intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, LegalState.NEUTRAL, arm ,intake)});
    }
  }

  private class PlaceHatchRocketMiddleForward extends PlaceGrabState{
    private PlaceHatchRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(47.8, 35, 11.7, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD, arm ,intake)}, true, 6.064, false);
    }
  }

  private class ReadyPlaceHatchRocketMiddleBack extends PlaceGrabState{
    private ReadyPlaceHatchRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(-36.3, -57, 9.3, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK, arm ,intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.NEUTRAL, arm, intake) ,
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.GRAB_BALL_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake) 
      }, false, 28.0, false);
    }
  }

  private class PlaceHatchRocketMiddleBack extends PlaceGrabState{
    private PlaceHatchRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(-41.5, -51, 9.5, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm ,intake)}, false, 25.0, true);
    }
  }

  private class ReadyPlaceBallRocketLowForward extends ArmState{
    private ReadyPlaceBallRocketLowForward(Arm arm, BaseIntake intake) {
      super(80, 6, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, LegalState.PLACE_BALL_ROCKET_LOW_FORWARD, arm ,intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, LegalState.NEUTRAL, arm ,intake)
      });
    }
  }

  private class PlaceBallRocketLowForward extends PlaceGrabState{
    private PlaceBallRocketLowForward(Arm arm, BaseIntake intake) {
      super(82.5, 6, 7.1, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_LOW_FORWARD, LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD, arm ,intake)}, true, 11.042, true);
    }
  }

  private class ReadyPlaceBallRocketLowBack extends ArmState {
    private ReadyPlaceBallRocketLowBack(Arm arm, BaseIntake intake) {
      super(-78, -12, 0.33, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.PLACE_BALL_ROCKET_LOW_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.NEUTRAL, arm, intake) ,
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.GRAB_BALL_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake) 
      });
    }
  }

  private class PlaceBallRocketLowBack extends PlaceGrabState {
    private PlaceBallRocketLowBack(Arm arm, BaseIntake intake) {
      super(-78, -10, 4, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_LOW_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake)}, false, 31.216, true);
    }
  }
  
  // // not needed because shooting ball into middle port of rocket
  // private class ReadyPlaceBallRocketMiddleForward extends ArmState {
  //   private ReadyPlaceBallRocketMiddleForward(Arm arm, BaseIntake intake) {
  //     super(0, 25, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, arm, intake), 
  //       new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD, LegalState.NEUTRAL, arm, intake) });
  //   }
  // }

  // shooting needs testing
  private class PlaceBallRocketMiddleForward extends PlaceGrabState {
    private PlaceBallRocketMiddleForward(Arm arm, BaseIntake intake) {
      super(26.6, 48.4, 12.7306, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD, LegalState.NEUTRAL, arm, intake)}, true, 35.0, false);
    }
  }

  private class ReadyPlaceBallRocketMiddleBack extends ArmState {
    private ReadyPlaceBallRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(-11.791, -48.0, 11.9848, false, new ArmTransition[] { new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.NEUTRAL, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.READY_GRAB_HATCH_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.GRAB_BALL_GROUND_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.GRAB_BALL_LOADINGSTATION_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.READY_LOW_HATCH_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.PLACE_BALL_CARGOSHIP_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK, arm, intake), 
        new CommonTransition(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK, arm, intake) 
      });
    }
  }

  private class PlaceBallRocketMiddleBack extends PlaceGrabState {
    private PlaceBallRocketMiddleBack(Arm arm, BaseIntake intake) {
      super(-16.977, -43.0, 12.7306, false, new ArmTransition[] { new CommonTransition(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK, LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK, arm, intake)}, false, 18.184, true);
    }
  }

  private class ReadyToClimb extends ArmState {
    private ReadyToClimb(Arm arm, BaseIntake intake) {
      super(0, 29, 0, false, new ArmTransition[] { new CommonTransition(LegalState.READY_TO_CLIMB, LegalState.NEUTRAL, arm, intake)});
    }
  }
  private class Stow extends ArmState {
      private Stow(Arm arm, BaseIntake intake) {
        super(103, -105, 0, false, new ArmTransition[] { new CommonTransition(LegalState.STOW, LegalState.NEUTRAL, arm, intake)});
      } 
  }

  public void setCoastMode(){
    getElbow().enableCoastMode();
    getElbow().stop();
    getWrist().enableCoastMode();
    getWrist().stop();
    getExtension().enableCoastMode();
    getExtension().stop();
  }

  public void setBrakeMode(){
    getElbow().enableBrakeMode();
    getElbow().stop();
    getWrist().enableBrakeMode();
    getWrist().stop();
    getExtension().enableBrakeMode();
    getExtension().stop();
  }

  public Claw getClaw(){
    return claw;
  }
}
