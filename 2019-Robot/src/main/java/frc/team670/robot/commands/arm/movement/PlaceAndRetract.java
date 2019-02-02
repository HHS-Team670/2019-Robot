/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import javax.management.modelmbean.ModelMBeanOperationInfo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PlaceAndRetract extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PlaceAndRetract(String target, String height) {
    // if placing hatch on rocket
    if (target.contains("Rocket") && (target.contains("1") || target.contains("3"))) {
      if (height.equals("MIDDLE")) {

      } else if (height.equals("LOW")) {
        
      }
    }
    // if placing ball on rocket
    else if (target.contains("Rocket") && target.contains("2")) {
      if (height.equals("MIDDLE")) {

      } else if (height.equals("LOW")) {
        
      }
    }
    // if placing anything in cargo ship
    else if (target.contains("Cargo")) {
      // if placing ball
      if (height.equals("MIDDLE")) {

      } 
      // if placing hatch
      else if (height.equals("LOW")) {

      }
    }

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
