/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.HashMap;

import org.junit.Test;

import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;

/**
 * Tests the MoveArm Command by running through all ArmStates and ensuring they pathfind to the proper position by testing
 * their end positions compared to the position of the destination State.
 */
public class MoveArmTest {

    @Test
    public void testMoveArm() {

        TestElbow elbow = new TestElbow();
        TestWrist wrist = new TestWrist();
        TestExtension extension = new TestExtension();
        Arm arm = new Arm(elbow, wrist, extension);

        HashMap<LegalState, ArmState> armStates = Arm.getStates();

        ArrayList<ArmState> states = new ArrayList<ArmState>(armStates.values());

        for(ArmState startState : states) {

            ArmTransition[] transitions = startState.getEdges();

            for(ArmTransition transition : transitions) {                
                elbow = new TestElbow();
                wrist = new TestWrist();
                extension = new TestExtension();


                ArmState dest = transition.getDest();
                double finalElbowAngle = dest.getElbowAngle();
                double finalWristAngle = dest.getWristAngle();
                double finalExtensionLength = dest.getExtensionLength();

                Scheduler.getInstance().add(transition);
                while(!transition.isCompleted()) {
                    Scheduler.getInstance().run();
                }

                System.out.println(finalElbowAngle);
                System.out.println(finalWristAngle);
                System.out.println(finalExtensionLength);
                
                assertEquals(finalElbowAngle, elbow.getAngle(), 0.00001);
                assertEquals(finalWristAngle, wrist.getAngle(), 0.00001);
                assertEquals(finalExtensionLength, extension.getLengthInches(), 0.00001);
                assertEquals(dest.getCoordPosition(), Arm.getCoordPosition(elbow.getAngle(), wrist.getAngle(), extension.getLengthInches()));
                assertEquals(dest, Arm.getCurrentState());

            }

        }
    }

}
