/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import java.util.List;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.sort.AStarSearch;

/**
 * Move the arm if it is at a known ArmState by finding a path and using it to travel along ArmTransitions between ArmStates.
 * @author ctchen
 */
public class ArmPathGenerator {
  // private static Map<ArmState, List<ArmTransition>> searched = new HashMap<ArmState, List<ArmTransition>>();

  private ArmPathGenerator() {

  }

  /** 
   * Looks for an existing path. If none exists, search for the path to move to.
   * Called just before this Command runs the first time
   * */
  public static CommandGroup getPath(ArmState destination, Arm arm) {
    ArmState currentState = Arm.getCurrentState();
    Logger.consoleLog("currentState: %s, destinationState: %s", currentState.getClass().getName(), destination.getClass().getName());
    CommandGroup movements = new CommandGroup() {
      @Override
      protected void end() { // The created path will set the Arm's state to wherever it moved after finishing
        Arm.setState(destination);
      }

      @Override
        protected void interrupted() {
          end();
          Logger.consoleLog("Interrupted");
        }
    };
    // List<ArmTransition> transitions = searched.get(currentState);
    List<ArmTransition> transitions = null;
    try {
      transitions = (List<ArmTransition>)(List<?>)(AStarSearch.search(currentState, destination));
    } catch(ClassCastException e) {
      Logger.logException(e);
      Logger.consoleLog("Edge passed in was not an ArmTransition. Command canceling");
      movements.cancel();
      return movements;
    } catch (IllegalArgumentException e) {
      // Logger.logException(e);
      Logger.consoleLog("Passed in bad argument");
      movements.cancel();
      return movements;
    // searched.put(currentState, transitions); //Stores current path in instance variable
    }
    for (ArmTransition t : transitions) {
      movements.addSequential(t);
    }

    return movements;
  }

  // // Called once after isFinished returns true
  // @Override
  // protected void end() {
  //   Arm.setState(destination);
  // }

}
