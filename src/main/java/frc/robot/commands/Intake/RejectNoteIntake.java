package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RejectNoteIntake extends Command {
    Intake intake;

    public RejectNoteIntake(Intake subsystem) {
        intake = subsystem;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.rejectNote();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
         return false;
    }
}
