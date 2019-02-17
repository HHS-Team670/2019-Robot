package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.team670.robot.Robot;
/**
 *  import com.pi4j.io.gpio.*;
 *  ^ importing this would be useful
 */


public class DIOUltrasonic {

    private DigitalOutput triggerPin;
    private DigitalInput echoPin;
    private Ultrasonic ultrasonic;
    public static final double ULTRASONIC_ERROR_CODE = 99999;

    private double horizontalOffset; // horizontal offset from the center of the robot on the side it is on. Left is negative, right is positive.

    /**
     * @param horizontalOffset horizontal offset from the center of the robot on the side it is on. Left is negative, right is positive.
     */
    public DIOUltrasonic(int tPin, int ePin, double horizontalOffset){
        triggerPin = new DigitalOutput(tPin);
        echoPin = new DigitalInput(ePin);

        this.horizontalOffset = horizontalOffset;

        try {
             ultrasonic =  new Ultrasonic(triggerPin, echoPin);
        } catch (RuntimeException ex){
            ultrasonic = null;
            DriverStation.reportError("Ultrasonic error", true);
        }

        ultrasonic.setAutomaticMode(true);
    }

    /**
     * Gets the ultrasonic distance in inches adjusted for the angle to target and offset of the ultrasonic from the center of the robot.
     * 
     * @param angle The angle to the target from the robot.
     */
    public double getDistance(double angle){
        double distance = getUnadjustedDistance();
        // Untested Math below
        double phi = Robot.sensors.getAngleToTarget();
        distance = horizontalOffset * Math.tan(Math.toRadians(phi)) + distance;
        return distance;
    }

    /**
     * Gets the ultrasonic distance unadjusted for offset and angle to target
     */
    public double getUnadjustedDistance() {
        if (ultrasonic != null){
        return ultrasonic.getRangeInches();
        }   else {
            return ULTRASONIC_ERROR_CODE;
        }
    }


} 