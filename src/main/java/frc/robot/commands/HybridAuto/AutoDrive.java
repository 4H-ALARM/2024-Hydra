package frc.robot.commands.HybridAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hybrid.ControlVector;

public class AutoDrive extends Command {

    private final AutoVectors robotVectors;
    private final AutoVectors setVectors;
    public AutoDrive(AutoVectors robotVectors, AutoVectors setVectors) {
        this.robotVectors = robotVectors;
        this.setVectors = setVectors;
    }

    @Override
    public void initialize() {
        this.robotVectors.setFrom(setVectors);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        robotVectors.setZero();
    }
}
