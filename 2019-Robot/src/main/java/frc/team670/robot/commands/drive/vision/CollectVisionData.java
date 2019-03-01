/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Starts a Pure Pursuit path based off vision data
 */
public class CollectVisionData extends Command {
    private double[] visionData;
    private long startTime;

  public CollectVisionData(double[] visionData) {
    super();
    this.visionData = visionData;

  }

  public void initialize(){
      startTime = System.currentTimeMillis();
  }

  public boolean isFinished(){
      return (!MathUtils.doublesEqual(visionData[2], RobotConstants.VISION_ERROR_CODE) || System.currentTimeMillis() - startTime > 1200);
  }

  public void execute(){
      long time = System.currentTimeMillis();

      if(time >= startTime + 500){
          visionData = Robot.coprocessor.getVisionValues();
      }

  }

}
