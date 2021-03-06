/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.sort.Edge;

  /**
   * The base for arm transitions. All iterations of this class should be made so that they can be rerun each time initialize is called.
   * Essentially, initialize should reset all necessary values within the object.
   */
public abstract class ArmTransition extends CommandGroup implements Edge {

  private LegalState source, dest;

  private Arm arm;
  protected BaseElbow elbow;
  protected BaseWrist wrist;
  protected BaseExtension extension;
  protected BaseIntake intake;

  /**
   * 
   * 
   * Because I do not want to go through the trouble of making a TestClaw right now, the ArmTransition and ArmMovement tests are commented out.
   * so this will build properly. These tests have always passed before, but if you add new transitions these need to be added back.
   * 
   * READ THIS COMMMMMMEEEEENTTTTTTTT
   * 
   */


  protected ArmTransition(LegalState source, LegalState dest, Arm arm, BaseIntake intake) {
    this.source = source;
    this.dest = dest;
    elbow = arm.getElbow();
    wrist = arm.getWrist();
    extension = arm.getExtension();
    this.intake = intake;
    this.arm = arm;
  }

  

  @Override
  protected void initialize() {
    Arm.setState(getDest());
  }
  
  /**
   * The LegalState that this Command will end at.
   */
  @Override
  public ArmState getSource() {
    return Arm.getArmState(source);
  }

  /**
   * Example for a specified transition where you would need to take certain steps to optimize it.
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
   
  }

  public abstract void initTransition();

  public abstract CommandGroup getCommand();

}