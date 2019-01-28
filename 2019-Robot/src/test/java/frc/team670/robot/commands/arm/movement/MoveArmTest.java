// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.team670.robot.commands.arm.movement;

// import static org.junit.Assert.assertEquals;

// import org.junit.Test;

// import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.command.Scheduler;
// import frc.team670.robot.subsystems.Arm;
// import frc.team670.robot.subsystems.Arm.ArmState;
// import frc.team670.robot.subsystems.Arm.LegalState;
// import frc.team670.robot.utils.Logger;

// /**
//  * Add your docs here.
//  */
// public class MoveArmTest {

//     @Test
//     public void testMoveArm() {

//         TestElbow elbow = new TestElbow();
//         TestWrist wrist = new TestWrist();
//         TestExtension extension = new TestExtension();
//         Arm arm = new Arm(elbow, wrist, extension);
//         Arm.setState(Arm.getArmState(LegalState.NEUTRAL));
//         ArmState dest = Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_LOW_FORWARD);

//         CommandGroup moveArm = ArmPathGenerator.getPath(dest, arm);

//         Scheduler.getInstance().add(moveArm);  
//         moveArm.setRunWhenDisabled(true); // Must be true or it won't run       
//         moveArm.start();   
//         while(!moveArm.isCompleted()) {
//             Scheduler.getInstance().run();
//         }
        
//         assertEquals(dest.getCoordPosition(), Arm.getCoordPosition(elbow.getAngle(), wrist.getAngle(), extension.getLengthInches()));

//         Arm.setState(Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_LOW_FORWARD));
//         dest = Arm.getArmState(LegalState.NEUTRAL);
//         moveArm = ArmPathGenerator.getPath(dest, arm);
        
//         Scheduler.getInstance().add(moveArm);  
//         moveArm.setRunWhenDisabled(true); // Must be true or it won't run       
//         moveArm.start();   
//         while(!moveArm.isCompleted()) {
//             Scheduler.getInstance().run();
//         }
//         assertEquals(dest.getCoordPosition(), Arm.getCoordPosition(elbow.getAngle(), wrist.getAngle(), extension.getLengthInches()));

//     }

// }
