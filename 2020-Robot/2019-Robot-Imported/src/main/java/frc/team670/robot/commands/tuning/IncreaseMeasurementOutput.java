/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class IncreaseMeasurementOutput extends InstantCommand {
  /**
   * Add your docs here.
   */
  public IncreaseMeasurementOutput() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    MeasureArbitraryFeedforward.output += 0.005;
    SmartDashboard.putNumber("ArbitraryFeedForwardOutput", MeasureArbitraryFeedforward.output);
  }

}
