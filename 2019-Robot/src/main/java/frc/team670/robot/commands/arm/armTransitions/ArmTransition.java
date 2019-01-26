/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.elbow.Elbow;
import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.sort.Edge;

  /**
   * The base for arm transitions. All iterations of this class should be made so that they can be rerun each time initialize is called.
   * Essentially, initialize should reset all necessary values within the object.
   */
public abstract class ArmTransition extends CommandGroup implements Edge {

  private LegalState source, dest;

  protected Elbow elbow;
  protected Wrist wrist;
  protected Extension extension;

  protected ArmTransition(LegalState source, LegalState dest, Arm arm) {
    this.source = source;
    this.dest = dest;
  }
  
  /**
   * The LegalState that this Command will end at.
   */
  @Override
  public ArmState getSource() {
    return Arm.getArmState(source);
  }

  /**
   * The LegalState that this Command must begin at.
   */
  @Override
  public ArmState getDest() {
    return Arm.getArmState(dest);
  }

  /**
   * Returns the sum of the number of inches that the arm must move to travel the path.
   */
  public int getCost() {
    Point2D.Double sourceCoord = getSource().getCoordPosition(), destCoord = getDest().getCoordPosition();
    return (int)(MathUtils.findDistance(sourceCoord.x, sourceCoord.y, destCoord.x, destCoord.y));
  }

  @Override
  protected void end() {
    Arm.setState(getDest());
  }

}





/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package frc.team670.robot.commands.arm.armTransitions;

// import edu.wpi.first.wpilibj.command.CommandGroup;
// import frc.team670.robot.subsystems.Arm.LegalState;
// import frc.team670.robot.utils.sort.Edge;
// import frc.team670.robot.commands.arm.MoveArm;

  /**
   * The base for arm transitions. All iterations of this class should be made so that they can be rerun each time initialize is called.
   * Essentially, initialize should reset all necessary values within the object.
   */

// public abstract class ArmTransition extends CommandGroup implements Edge {

//   private final int GroundPosition = 0;
//   private double hatchPanel1, hatchPanel2, hatchPanel3;
//   private double port1, port2, port3;

//   hatchPanel1 = 16.5;
//   hatchPanel2 = 54.5;
//   hatchPanel3 = 82.5;
  
//   port1 = 27.5;
//   port2 = 55.5;
//   port3 = 83.5;
    
  /**
   * The LegalState that this Command should begin at.
   */
 // public abstract LegalState getStart();
    
  /**
   * The LegalState that this Command will end at.
   */
 // public abstract LegalState getDestination();  

  /**
   * Returns the sum of the number of inches that the arm must move to cover the path to use as a method of weighting the transition.
   * Ticks that are traveled parallel to each other should not be counted. So if the arm is moving two joints at once, it should only
   * count the longer of the movements. Eventually, we can change this method to time spent once we have to robot built and can run
   * some empirical tests of the mechanisms.
   */
  /*public abstract int getLength();
   
}
}
*/