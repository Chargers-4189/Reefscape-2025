package frc.util;

import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {
    private double initTime;
    private double timeout;
    private double duration;

    public Stopwatch(double seconds){
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + seconds;
        duration = seconds;
    }

    public boolean hasTriggered(){
        initTime = Timer.getFPGATimestamp();
        return initTime > timeout;
    }

    public void initStopwatch(){
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + duration;
    }
}
