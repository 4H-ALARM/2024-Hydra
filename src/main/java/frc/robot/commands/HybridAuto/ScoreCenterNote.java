package frc.robot.commands.HybridAuto;

import frc.robot.hybrid.ControlVector;

public class ScoreCenterNote extends HybridAutoBaseCommand {
    public ScoreCenterNote(ControlVector driveVector, ControlVector intakeAimInfluence, ControlVector shootAimInfluence) {
        super(driveVector, intakeAimInfluence, shootAimInfluence);
    }

    @Override
    public void initialize() {
        // INCORRECT: this.driveVector = new ControlVector().setSwerveRobotY(0.5);
        this.driveVector.setSwerveRobotY(0.5);
        this.intakeAimInfluence.setSwerveRotation(0.3);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        driveVector.setSwerveRobotY(0);
        intakeAimInfluence.setSwerveRotation(0);
    }
}
