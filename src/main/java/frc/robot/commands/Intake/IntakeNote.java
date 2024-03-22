package frc.robot.commands.Intake;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    Intake intake;

    public IntakeNote(Intake subsystem) {
        intake = subsystem;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakeMotor(false);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return intake.isIntaked();
    }

}
