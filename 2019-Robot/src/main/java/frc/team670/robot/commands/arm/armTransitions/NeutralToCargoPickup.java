/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import frc.team670.robot.subsystems.Arm.LegalState;

public class NeutralToCargoPickup extends ArmTransition {
  
  @Override
  public LegalState getDestination() {
    return LegalState.CARGO_PICKUP;
  }
  
  @Override
  public LegalState getStart() {
    return LegalState.NEUTRAL;
  }

  @Override
  public int getLength() {
    return 0; // Set this
  }

  /**
   * Add your docs here.
   */
  public NeutralToCargoPickup() {
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
