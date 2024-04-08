package frc.robot.classes;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon2Handler {
    private final Pigeon2 pigeon;

    public Pigeon2Handler(int CanID) {
        this(
            CanID,
            null
        );
        
    }

    public Pigeon2Handler(int CanID, String Canbus) {
        this.pigeon = new Pigeon2(CanID, Canbus);
        pigeon.clearStickyFaults();
    }

    public double getUnwraappedAngle() {
        return pigeon.getAngle();
    }

    public double getWrappedAngle() {
        double unwrappedangle = pigeon.getAngle();
        double angle = pigeon.getAngle();
        if (unwrappedangle >360) {
            angle = unwrappedangle - (360 * (unwrappedangle/360));
        }
        return angle;
    }

}