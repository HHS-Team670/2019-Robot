/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * Add your docs here.
 */
public class MoveArm extends InstantCommand {
 
  private ArmState destination;
  private Arm arm;

  public MoveArm(ArmState destination, Arm arm) {
    super();
    this.destination = destination;
    this.arm = arm;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {

    CommandGroup moveArm = ArmPathGenerator.getPath(destination, arm);
    // TODO add stuff in here to store the path made based on the currentState and then do a lookup instead of a new search
    Scheduler.getInstance().add(moveArm);
    
  }

}
