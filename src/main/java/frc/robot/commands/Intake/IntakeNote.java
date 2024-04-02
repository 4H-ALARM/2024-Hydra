package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    Intake intake;
    private double speed;

    public IntakeNote(Intake subsystem) {
        this(subsystem, 0);
    }

    public IntakeNote(Intake subsystem, double speed) {
        intake = subsystem;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.pickUpNote();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.finishedIntaking();
    }

}
