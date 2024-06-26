package frc.robot.commands.CommandGroups.ShootCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Arm.ShootPosition;
import frc.robot.commands.Shooter.RevShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PrepareShootCommandGroup extends SequentialCommandGroup {
    public PrepareShootCommandGroup(Arm arm, Indexer indexer, Intake intake, Shooter shooter, CommandXboxController controller) {
        super(new RevShooter(shooter, controller));
    }
}
