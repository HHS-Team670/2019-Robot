/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import static org.junit.Assert.assertEquals;

import java.awt.geom.Point2D;

import org.junit.Test;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Add your docs here.
 */
public class MoveArmTest {

    @Test
    public void testMoveArm() {

        TestElbow elbow = new TestElbow();
        TestWrist wrist = new TestWrist();
        TestExtension extension = new TestExtension();
        TestIntake intake = new TestIntake();
        Claw claw = new Claw();
        Arm arm = new Arm(elbow, wrist, extension, intake, claw);
        
        for(ArmState state : Arm.getStates().values()) {
            for(ArmState dest : Arm.getStates().values()) {

                if(!dest.equals(Arm.getArmState(LegalState.START))) {
                    Command moveArmToStart = new MoveArmDangerous(state, arm, intake);
                    Scheduler.getInstance().add(moveArmToStart);  
                    moveArmToStart.setRunWhenDisabled(true); // Must be true or it won't run       
                    moveArmToStart.start();   
                    while(!moveArmToStart.isCompleted()) {
                        Scheduler.getInstance().run();
                    }

                    CommandGroup moveArm = ArmPathGenerator.getPath(dest, arm);

                    Scheduler.getInstance().add(moveArm);  
                    moveArm.setRunWhenDisabled(true); // Must be true or it won't run       
                    moveArm.start();   
                    while(!moveArm.isCompleted()) {
                        Scheduler.getInstance().run();
                    }
            
                    Point2D.Double armCoord = Arm.getCoordPosition(elbow.getAngleInDegrees(), wrist.getAngleInDegrees(), extension.getLengthInches());
                    assertEquals(dest, Arm.getCurrentState());
                    try {
                        // System.out.println("start: " + state.getClass().getName() + ", dest: " + dest.getClass().getName());
                        assertEquals(dest.getCoordPosition().x, armCoord.x, 0.5);
                        assertEquals(dest.getCoordPosition().y, armCoord.y, 0.5); 
                    }
                    catch(OutOfMemoryError e) {
                        throw new AssertionError("Start: " + state.getClass().getName() + ", Dest: " + dest.getClass().getName());
                    }
                    double expectedIntakeAngle;
                    if(dest.isIntakeDeployed()) {
                        expectedIntakeAngle = Intake.INTAKE_ANGLE_DEPLOYED;
                    } else {
                        expectedIntakeAngle = Intake.INTAKE_ANGLE_IN;
                    }
                    assertEquals(true, MathUtils.isWithinTolerance(expectedIntakeAngle, intake.getAngleInDegrees(), 0.5));
                }
            }
        }

        for(ArmState state : Arm.getStates().values()) {
            for(ArmState dest : Arm.getStates().values()) {

                if(!dest.equals(Arm.getArmState(LegalState.START))) {
                    Command moveArmToStart = new MoveArmDangerous(state, arm, intake);
                    Scheduler.getInstance().add(moveArmToStart);  
                    moveArmToStart.setRunWhenDisabled(true); // Must be true or it won't run       
                    moveArmToStart.start();   
                    while(!moveArmToStart.isCompleted()) {
                        Scheduler.getInstance().run();
                    }

                    CommandGroup moveArm = ArmPathGenerator.getPath(dest, arm);

                    Scheduler.getInstance().add(moveArm);  
                    moveArm.setRunWhenDisabled(true); // Must be true or it won't run       
                    moveArm.start();   
                    while(!moveArm.isCompleted()) {
                        Scheduler.getInstance().run();
                    }
            
                    Point2D.Double armCoord = Arm.getCoordPosition(elbow.getAngleInDegrees(), wrist.getAngleInDegrees(), extension.getLengthInches());
                    assertEquals(dest, Arm.getCurrentState());
                    try {
                        // System.out.println("start: " + state.getClass().getName() + ", dest: " + dest.getClass().getName());
                        assertEquals(dest.getCoordPosition().x, armCoord.x, 0.5);
                        assertEquals(dest.getCoordPosition().y, armCoord.y, 0.5); 
                    }
                    catch(OutOfMemoryError e) {
                        throw new AssertionError("Start: " + state.getClass().getName() + ", Dest: " + dest.getClass().getName());
                    }
                    double expectedIntakeAngle;
                    if(dest.isIntakeDeployed()) {
                        expectedIntakeAngle = Intake.INTAKE_ANGLE_DEPLOYED;
                    } else {
                        expectedIntakeAngle = Intake.INTAKE_ANGLE_IN;
                    }
                    assertEquals(true, MathUtils.isWithinTolerance(expectedIntakeAngle, intake.getAngleInDegrees(), 0.5));
                }
            }
        }
    }

}
