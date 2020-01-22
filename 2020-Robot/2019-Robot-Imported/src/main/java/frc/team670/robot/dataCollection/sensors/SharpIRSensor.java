package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
import edu.wpi.first.wpilibj.Counter;

/**
 * yes
 * Driver for an analog Sharp IR sensor (or any distance sensor where output voltage is a function of range, really).
 */
public class SharpIRSensor {
    protected final AnalogInput M_ANALOG_INPUT;
    protected final AnalogTrigger M_ANALOG_TRIGGER;
    protected final Counter M_COUNTER;

    public SharpIRSensor(int port, double min_trigger_voltage, double max_trigger_voltage) {
        M_ANALOG_INPUT = new AnalogInput(port);
        M_ANALOG_INPUT.setAverageBits(6);
        M_ANALOG_TRIGGER = new AnalogTrigger(M_ANALOG_INPUT);
        M_ANALOG_TRIGGER.setAveraged(true);
        M_ANALOG_TRIGGER.setFiltered(false);
        M_ANALOG_TRIGGER.setLimitsVoltage(min_trigger_voltage, max_trigger_voltage);
        M_COUNTER = new Counter(M_ANALOG_TRIGGER.createOutput(AnalogTriggerType.kState));
    }

    public int getCount() {
        return M_COUNTER.get();
    }

    public double getVoltage() {
        return M_ANALOG_INPUT.getAverageVoltage();
    }

   // public boolean seesTape(){

    //}


    public void resetCount() {
        M_COUNTER.reset();
    }
}
