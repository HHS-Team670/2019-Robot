/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.commands.climb.pistonClimb.AbortRobotPistonClimb;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Climber;

public class CancelClimbBasedOnTimeLeftInMatch extends InstantCommand {
  private Arm arm;
  private Climber climber;
  private DriverStation ds;

  private static final int MIN_TIME_FOR_CLIMB = 5;

  public CancelClimbBasedOnTimeLeftInMatch(Arm arm, Climber climber) {
    this.climber = climber;
    this.arm = arm;
    ds = DriverStation.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(ds.getMatchTime() <= MIN_TIME_FOR_CLIMB && !climber.getFrontPistonsRetracted()){
      Scheduler.getInstance().add(new AbortRobotPistonClimb(climber, arm));
    }
  }
}
