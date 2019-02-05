package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.drive.FlipCamera;
import frc.team670.robot.commands.drive.FlipDriveDirection;

public class FlipDriveWithCamera extends CommandGroup{

    public FlipDriveWithCamera() {
        addSequential(new FlipCamera());
        addSequential(new FlipDriveDirection());
    }

}