package frc.team670.robot.commands.cameras;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;

/**
 *  Flips the camera: The front camera or the camera on the back
 */
public class FlipCamera extends InstantCommand{

    /**
     * Driver camera direction (true = front, false = back). Starts false because robot starts backwards.
     */
    private static boolean cameraDirection = false;

    public FlipCamera(){
        super();
    }

    // called once when the command executes
    @Override
    protected void initialize(){
        flipCameraDirection();
        SmartDashboard.putString("current-command", "FlipCamera");
        Robot.oi.rumbleDriverController(0.4, 0.1);
    }

    public static void flipCameraDirection() {
        int camNumber = 0;
        if (cameraDirection) camNumber = 0;
        else camNumber = 1;
        SmartDashboard.putString("camera-source", camNumber+"");
        cameraDirection = !cameraDirection;
    }

    /**
     * Driver camera direction (true = front, false = back). Starts false because robot starts backwards.
     */
    public static boolean getCameraDirection() {
        return cameraDirection;
    }
}