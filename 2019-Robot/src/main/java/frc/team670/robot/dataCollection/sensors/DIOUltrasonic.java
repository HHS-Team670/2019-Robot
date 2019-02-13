package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
/**
 *  import com.pi4j.io.gpio.*;
 *  ^ importing this would be useful
 */


public class DIOUltrasonic {
    private static DigitalOutput triggerPin;
    private static DigitalInput echoPin;
    private static Ultrasonic WPIUltrasonic;

    public DIOUltrasonic(){
        this(8,9); // Temporary values for Ultrasonic - move to RobotMap unless we want it like this for multiple sensors
    }

    public DIOUltrasonic(int tPin, int ePin){
        triggerPin = new DigitalOutput(tPin);
        echoPin = new DigitalInput(ePin);

        WPIUltrasonic =  new Ultrasonic(triggerPin, echoPin);

        WPIUltrasonic.setAutomaticMode(true);
    }

    public double getUltrasonicValue(){
        return WPIUltrasonic.getRangeInches();
    }


    public Ultrasonic getWPIUltrasonicObject(){
        return WPIUltrasonic;
    }
} 