/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import static org.junit.Assert.assertEquals;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.junit.Test;

import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * Tests the MoveArm Command by running through all ArmStates and ensuring they pathfind to the proper position by testing
 * their end positions compared to the position of the destination State.
 */
public class TestMoveArm {

    @Test
    public void testMoveArm(ArrayList<ArmState> states) {

        MoveArm moveArm;

        for(ArmState startState : states) {

            for(ArmState destState : states) {

                Arm.setState(startState);
                moveArm = new MoveArm(destState);
                moveArm.initialize();
                for(int i = 0; i < 25; i++) {
                    Scheduler.getInstance().run();
                }

                // Point2D.Double actualDestCoord = Arm.getCoordPosition(, wristAngle, elbowAngle);

                // assertEquals(destState.getCoordPosition().getX(), Arm.getCoordPosition(extensionLength, wristAngle, elbowAngle), 0.0001);
            }

        }

    }



}
