package frc.util;

import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {
    private double initTime;
    private double timeout;

    public Stopwatch() {
    }

    public boolean hasTriggered() {
        initTime = Timer.getFPGATimestamp();
        return initTime > timeout;
    }

    public void start(int milliseconds) {
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + (milliseconds * 1000);
    }
}
