/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Elbow;
import frc.team670.robot.subsystems.Extension;
import frc.team670.robot.subsystems.Wrist;

/**
 * Uses MotionMagic to move the Arm directly to a known ArmState. BE VERY CAREFUL WITH THIS. It does not take into account
 * any other position of the robot, so it will very easily slam into the intake or the climber pistons!!!!!
 * Can be useful if trying to move back to an ArmState that is out of the way of Robot parts when you know you are near it.
 * This should not be used to reset the encoders by hitting the limit switches because it requires the encoders to be accurately absolute.
 * @author ctchen, arleenliu
 */
public class MoveArmDangerous extends CommandGroup {

  private ArmState targetState;

  private Elbow elbow;
  private Wrist wrist;
  private Extension extension;

  public MoveArmDangerous(ArmState state, Arm arm) {
    super();
    elbow = arm.getElbow();
    wrist = arm.getWrist();
    extension = arm.getExtension();
    requires(extension);
    requires(wrist);
    requires(elbow);

    addParallel(new MoveExtension(extension, targetState.getExtensionLength()));
    addParallel(new MoveWrist(wrist, targetState.getWristAngle()));
    addSequential(new MoveElbow(elbow, targetState.getElbowAngle()));
    addSequential(new WaitForChildren());
  }

}
