/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.commands.arm.armTransitions.NeutralToCargoPickup;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.sort.AStarSearch;
import frc.team670.robot.utils.sort.Node;

/**
 * Represents the arm mechanism on the robot. Link to a model of the arm:
 * https://a360.co/2TLH2NO
 * 
 * @author shaylandias
 */
public class Arm extends Subsystem {

  private TalonSRX translationMotor;
  private TalonSRX extensionMotor;
  private TalonSRX elbowRotationMain;
  private VictorSPX elbowRotationSlave;
  private TalonSRX wristRotation;
  // All of the states
  private HashMap<LegalState, ArmState> states;
  private static ArmState currentState;

  public Arm() {
    translationMotor = new TalonSRX(RobotMap.armTranslationMotor);
    extensionMotor = new TalonSRX(RobotMap.armExtensionMotor);
    elbowRotationMain = new TalonSRX(RobotMap.armElbowRotationMotorTalon);
    wristRotation = new TalonSRX(RobotMap.armWristRotation);

    elbowRotationSlave = new VictorSPX(RobotMap.armElbowRotationMotorVictor);
    elbowRotationSlave.set(ControlMode.Follower, elbowRotationMain.getDeviceID());
    currentState = new Neutral(); //Default state
    states = new HashMap<LegalState, ArmState>();
    states.put(LegalState.NEUTRAL, new Neutral());
    /*
     * Add in all of the states here.
     */

  }

  public static void setState(ArmState state){
    currentState = state;
  }

  public static ArmState getState(){
    return currentState;
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
  public class ArmState implements Node{
    private double elbowAngle, wristAngle;
    private double extensionLength;
    private Point2D.Double coord;
    private LegalState state;
    private double f, g; //TODO Put this in Node file
    private Node parent; //TODO Put this in Node file

    private ArmTransition[] transitions;

    /**
     * @param transitionableStates Should h
     */
    public ArmState(LegalState state, double extensionLength, double elbowAngle, double wristAngle, ArmTransition[] transitions) {
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

    //USE THIS
    public ArmTransition[] getEdges() {
      return transitions;
    }

    public int getHeuristicDistance(Node other){
      ArmState state2 = (ArmState)other;
      return (int)(Math.sqrt((this.coord.getX()-state2.coord.getX())*(this.coord.getX()-state2.coord.getX())+(this.coord.getY()-state2.coord.getY())*(this.coord.getY()-state2.coord.getY())));
    }


    public CommandGroup getTransition(ArmState destination) {
      CommandGroup result = new CommandGroup();

      for(ArmTransition transition : transitions) {
        if(transition.getDestination().equals(destination)) {
          result.addSequential(transition);
          return result;
        }
      }

      // get all states (nodes) from hashmap
      //Collection<ArmState> values = states.values();  
      //Creating an ArrayList of values        
      //ArrayList<ArmState> armStates = new ArrayList<ArmState>(values);
      ArmTransition[] armTransitions = (ArmTransition[])(AStarSearch.search((Node)(currentState), (Node)(destination)).toArray());

      /*
       * Implement this so it searches for the quickest path of transititons to the destination.
       * The A* Search Algorithm might be a good choice for this: https://www.geeksforgeeks.org/a-search-algorithm/ 
       * Otherwise you could potentially do a breadth-first search, but A* will work best if we weight certain transitions
       * for the time spent performing them. Currently this is measured as a tick count since we have no way of actually
       * timing it.
       * Long Stanford explanation on this: https://cs.stanford.edu/people/abisee/gs.pdf

        find minimum g+h 
        minimize distance
       */

      return result;
    }
  }

  private class Neutral extends ArmState {
    public Neutral() {
      super(LegalState.NEUTRAL, 0, 45, 45, new ArmTransition[] {new NeutralToCargoPickup()});
    }
  }

}
