package frc.team670.robot.commands.drive.teleop;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.cameras.FlipCamera;
import frc.team670.robot.utils.Logger;

public class FlipDriveAndCamera extends InstantCommand {

    public FlipDriveAndCamera() {
        super();
    }

    @Override
    protected void initialize() {
      SmartDashboard.putString("current-command", "FlipDriveAndCamera");
      boolean isReversed = XboxRocketLeagueDrive.isDriveReversed();

      // Matches camera direction to the new drive direction
        if(isReversed) {
            if(!FlipCamera.getCameraDirection()) {
                FlipCamera.flipCameraDirection();
            }
        } else {
            if(FlipCamera.getCameraDirection()) {
                FlipCamera.flipCameraDirection();
            }
        }

      if (!isReversed) {
        Robot.leds.setReverseData(true);
      } else {
        Robot.leds.setForwardData(true);
      }
      XboxRocketLeagueDrive.setDriveReversed(!isReversed);
      Robot.oi.rumbleDriverController(0.7, 0.2);
      Logger.consoleLog("Flipped Drive: %s", (!isReversed));
    }

}