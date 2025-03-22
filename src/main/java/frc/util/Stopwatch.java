package frc.util;

import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {
    private Double initTime = null;
    private Double timeout = null;

    public Stopwatch() {
    }

    public boolean hasTriggered() {
        if (!hasStarted()) {
            System.out.println("ERROR: Stopwatch not started!");
        }
        initTime = Timer.getFPGATimestamp();
        return initTime > timeout;
    }

    public void start(double milliseconds) {
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + (milliseconds / 1000.0);
    }
    
    public boolean hasStarted() {
        return timeout != null && initTime != null;
    }
    public void reset() {
        initTime = null;
        timeout = null;
    }
}
