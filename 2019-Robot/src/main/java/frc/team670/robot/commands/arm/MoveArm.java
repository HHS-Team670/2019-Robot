/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.utils.sort.AStarSearch;
import frc.team670.robot.utils.sort.Node;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Move the arm using known ArmStates and found path.
 * @author ctchen
 */
public class MoveArm extends CommandGroup {
  private CommandGroup movements;
  private ArmState destination;
  private Map<ArmState, List<ArmTransition>> searched = new HashMap<ArmState, List<ArmTransition>>();
  // private ArmState currentState;

  public MoveArm(ArmState destination) {
    this.destination = destination;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    ArmState currentState = Arm.getState();
    movements = new CommandGroup();
    List<ArmTransition> transitions = searched.get(currentState);
    if (transitions == null) {
      transitions = (List<ArmTransition>)(List<?>)(AStarSearch.search(currentState, destination));
      searched.put(currentState, transitions);
    }
    for (ArmTransition t : transitions){
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
