// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.team670.robot.commands.arm.movement;

// import static org.junit.Assert.assertEquals;

// import java.awt.geom.Point2D;
// import java.util.ArrayList;
// import java.util.HashMap;

// import org.junit.Test;

// import edu.wpi.first.wpilibj.command.Scheduler;
// import frc.team670.robot.commands.arm.armTransitions.ArmTransition;
// import frc.team670.robot.subsystems.Arm;
// import frc.team670.robot.subsystems.Intake;
// import frc.team670.robot.subsystems.Arm.ArmState;
// import frc.team670.robot.subsystems.Arm.LegalState;
// import frc.team670.robot.utils.functions.MathUtils;

// /**
//  * Tests the MoveArm Command by running through all ArmStates and ensuring they pathfind to the proper position by testing
//  * their end positions compared to the position of the destination State.
//  */
// public class ArmTransitionTest {

//     @Test
//     public void testArmTransitions() {

//         TestElbow elbow = new TestElbow();
//         TestWrist wrist = new TestWrist();
//         TestExtension extension = new TestExtension();
//         TestIntake intake = new TestIntake();
//         Arm arm = new Arm(elbow, wrist, extension, intake);

//         HashMap<LegalState, ArmState> armStates = Arm.getStates();

//         ArrayList<ArmState> states = new ArrayList<ArmState>(armStates.values());

//         for(ArmState startState : states) {

//             ArmTransition[] transitions = startState.getEdges();

//             for(ArmTransition transition : transitions) {                
//                 ArmState dest = transition.getDest();
//                 double finalElbowAngle = dest.getElbowAngle();
//                 double finalWristAngle = dest.getWristAngle();
//                 double finalExtensionLength = dest.getExtensionLength();


//                 Scheduler.getInstance().add(transition);  
//                 transition.setRunWhenDisabled(true); // Must be true or it won't run       
//                 transition.start();   
//                 while(!transition.isCompleted()) {
//                     Scheduler.getInstance().run();
//                 }

//                 double isIntakeDeployed;
//                 if(dest.isIntakeDeployed()) {
//                     isIntakeDeployed = Intake.INTAKE_ANGLE_DEPLOYED;
//                 } else {
//                     isIntakeDeployed = Intake.INTAKE_ANGLE_IN;
//                 }

                
//                 assertEquals(true, MathUtils.isWithinTolerance(isIntakeDeployed, intake.getIntakeAngleInDegrees(), 0.3));
//                 assertEquals(finalElbowAngle, elbow.getAngle(), 0.1);
//                 assertEquals(finalWristAngle, wrist.getAngle(), 0.1);
//                 assertEquals(finalExtensionLength, extension.getLengthInches(), 0.1);
//                 Point2D.Double armCoord = Arm.getCoordPosition(elbow.getAngle(), wrist.getAngle(), extension.getLengthInches());
//                 assertEquals(dest.getCoordPosition().x, armCoord.x, 0.2);
//                 assertEquals(dest.getCoordPosition().y, armCoord.y, 0.2);
//                 assertEquals(dest, Arm.getCurrentState());
//             }

//         }
//     }

// }