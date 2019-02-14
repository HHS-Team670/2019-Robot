package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
/**
 *  import com.pi4j.io.gpio.*;
 *  ^ importing this would be useful
 */


public class DIOUltrasonic {

    private DigitalOutput triggerPin;
    private DigitalInput echoPin;
    private Ultrasonic ultrasonic;

    private double horizontalOffset; // horizontal offset from the center of the robot on the side it is on. Left is negative, right is positive.

    public DIOUltrasonic(){
        this(8,9, 0); // Temporary values for Ultrasonic - move to RobotMap unless we want it like this for multiple sensors
    }

    /**
     * @param horizontalOffset horizontal offset from the center of the robot on the side it is on. Left is negative, right is positive.
     */
    public DIOUltrasonic(int tPin, int ePin, double horizontalOffset){
        triggerPin = new DigitalOutput(tPin);
        echoPin = new DigitalInput(ePin);

        this.horizontalOffset = horizontalOffset;

        ultrasonic =  new Ultrasonic(triggerPin, echoPin);

        ultrasonic.setAutomaticMode(true);
    }

    /**
     * Gets the ultrasonic distance in inches adjusted for the angle to target and offset of the ultrasonic from the center of the robot.
     * 
     * @param angle The angle to the target from the robot.
     */
    public double getDistance(double angle){

        double distance = ultrasonic.getRangeInches();
        // TODO do the math in here
        return distance;
    }

    /**
     * Gets the ultrasonic distance unadjusted for offset and angle to target
     */
    public double getUnadjustedDistance() {
        return ultrasonic.getRangeInches();
    }


} 