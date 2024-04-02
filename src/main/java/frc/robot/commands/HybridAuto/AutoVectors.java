package frc.robot.commands.HybridAuto;

import frc.robot.hybrid.ControlVector;

public class AutoVectors {
    public final ControlVector driveControl;
    public final ControlVector driveInfluence;
    public final ControlVector intakeInfluence;
    public final ControlVector shootInfluence;

    public AutoVectors(ControlVector driveControl, ControlVector driveInfluence, ControlVector intakeInfluence, ControlVector shootInfluence) {
        this.driveControl = driveControl;
        this.driveInfluence = driveInfluence;
        this.intakeInfluence = intakeInfluence;
        this.shootInfluence = shootInfluence;
    }

    public void setFrom(AutoVectors other) {
        this.driveControl.setFrom(other.driveControl);
        this.driveInfluence.setFrom(other.driveInfluence);
        this.intakeInfluence.setFrom(other.intakeInfluence);
        this.shootInfluence.setFrom(other.shootInfluence);
    }

    public  void setZero(){
        this.driveControl.setZero();
        this.driveInfluence.setZero();
        this.intakeInfluence.setZero();
        this.shootInfluence.setZero();
    }
}
