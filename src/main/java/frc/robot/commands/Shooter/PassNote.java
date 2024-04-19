package frc.robot.commands.Shooter;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PassNote extends Command {
    Shooter shooter;
    CommandXboxController copilot;

    public PassNote(Shooter subsystem, CommandXboxController copilot) {
        shooter = subsystem;
        this.copilot = copilot;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.passNoteShooter(this.copilot);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
