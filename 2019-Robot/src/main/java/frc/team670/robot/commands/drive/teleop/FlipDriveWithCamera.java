package frc.team670.robot.commands.drive.teleop;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.cameras.FlipCamera;

public class FlipDriveWithCamera extends CommandGroup{

    public FlipDriveWithCamera() {
        addSequential(new FlipCamera());
        addSequential(new FlipDriveDirection());
    }

}