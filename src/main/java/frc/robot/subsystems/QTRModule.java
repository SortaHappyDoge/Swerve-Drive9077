package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class QTRModule{
    DigitalOutput emitterPin;
    DigitalInput[] sensorArray;
    public QTRModule(int[] pinArray, int emitter){
        sensorArray = new DigitalInput[pinArray.length];
        int i = 0;
        for (int pin : pinArray) {
            sensorArray[i] = new DigitalInput(pin);
            ++i;
        }
        emitterPin = new DigitalOutput(emitter);
    }

    public boolean[] getState(){
        emitterPin.set(true);
        boolean[] states = new boolean[sensorArray.length];
        int i = 0;
        for (DigitalInput sensor : sensorArray) {
            states[i] = sensor.get();
            ++i;
        }
        emitterPin.set(false);
        return states;
    }
}
