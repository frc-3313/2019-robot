package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * DualValveSolenoid
 */
public class DualValveSolenoid {
    private Solenoid s0;
    private Solenoid s1;

    private int state = 2;

    DualValveSolenoid(int moduleNumber, int channel0, int channel1) {
        s0 = new Solenoid(moduleNumber, channel0);
        s1 = new Solenoid(moduleNumber, channel1);
    }

    public void set(int newState) {
        if (newState == 0) {
            s0.set(true);
            s1.set(false);
        } else if (newState == 1) {
            s0.set(false);
            s1.set(true);
        } else if (newState == 2) {
            s0.set(false);
            s1.set(false);
        }
    }

    public int get() {
        return state;
    }
}