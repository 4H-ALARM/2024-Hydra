package frc.robot.commands.Shooter;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
    Shooter shooter;

    public ShootAmp(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.startAMPShooter();
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
