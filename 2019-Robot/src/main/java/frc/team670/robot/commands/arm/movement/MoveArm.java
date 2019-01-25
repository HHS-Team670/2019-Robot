/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
public class MoveArm extends CommandGroup {
  private CommandGroup movements;
  private ArmState destination;
  private Map<ArmState, List<ArmTransition>> searched = new HashMap<ArmState, List<ArmTransition>>();
  private Arm arm;

  public MoveArm(ArmState destination, Arm arm) {
    this.destination = destination;
    this.arm = arm;
    requires(arm.getElbow());
    requires(arm.getWrist());
    requires(arm.getExtension());
  }

  /** 
   * Looks for an existing path. If none exists, search for the path to move to.
   * Called just before this Command runs the first time
   * */
  @Override
  protected void initialize() {
    ArmState currentState = Arm.getCurrentState();
    movements = new CommandGroup();
    List<ArmTransition> transitions = searched.get(currentState);
    if (transitions == null) {
      try {
        transitions = (List<ArmTransition>)(List<?>)(AStarSearch.search(currentState, destination));
      } catch(ClassCastException e) {
        Logger.logException(e);
        Logger.consoleLog("You really messed up.");
      }
      searched.put(currentState, transitions); //Stores current path in instance variable
    }
    for (ArmTransition t : transitions) {
      movements.addSequential(t);
    }
    movements.start(); // This might need to be Scheduler.addCommand(movements);

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Arm.setState(destination);
  }

}
