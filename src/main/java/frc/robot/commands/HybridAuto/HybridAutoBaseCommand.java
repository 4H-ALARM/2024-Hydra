package frc.robot.commands.HybridAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hybrid.ControlVector;

public class HybridAutoBaseCommand extends Command {
    protected final ControlVector driveVector;
    protected ControlVector intakeAimInfluence;
    protected ControlVector shootAimInfluence;

    public HybridAutoBaseCommand(ControlVector driveVector, ControlVector intakeAimInfluence, ControlVector shootAimInfluence) {
        this.driveVector = driveVector;
        this.intakeAimInfluence = intakeAimInfluence;
        this.shootAimInfluence = shootAimInfluence;
    }
}
