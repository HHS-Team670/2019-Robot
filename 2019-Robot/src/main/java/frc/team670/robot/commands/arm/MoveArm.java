/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.sort.AStarSearch;

/**
 * Move the arm using known ArmStates and found path.
 * @author ctchen
 */
public class MoveArm extends CommandGroup {
  private CommandGroup movements;
  private ArmState destination;
  private Map<ArmState, List<ArmTransition>> searched = new HashMap<ArmState, List<ArmTransition>>();

  public MoveArm(ArmState destination) {
    this.destination = destination;
    requires(Robot.arm);
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
    movements.start();

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Arm.setState(destination);
  }

}
