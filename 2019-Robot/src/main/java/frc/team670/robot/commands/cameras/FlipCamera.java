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
    private static int camNumber = 0;

    public FlipCamera(){
        super();
    }

    // called once when the command executes
    @Override
    protected void initialize(){
        flipCameraDirection();
        Robot.oi.rumbleDriverController(0.7, 0.2);
    }

    public static void flipCameraDirection() {
        cameraDirection = !cameraDirection;
        camNumber = (camNumber + 1) % 2;
        SmartDashboard.putString("camera-source", camNumber+"");
    }

    /**
     * Driver camera direction (true = front, false = back). Starts false because robot starts backwards.
     */
    public static boolean getCameraDirection() {
        return cameraDirection;
    }
}