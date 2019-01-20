package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *  Flips the camera: The front camera or the camera on the back
 */
public class FlipCamera extends InstantCommand{

    public FlipCamera(){
        super();
    }

    // called once when the command executes
    @Override
    protected void initialize(){
        // NetworkTable.putValue("/SmartDashboard/cameraSource", "next");
        // TODO Talk to Kishore about how top set this to flip the camera.
    }
}