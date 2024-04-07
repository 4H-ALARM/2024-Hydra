package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hybrid.BlendedControl;
import frc.robot.subsystems.Swerve;

public class HybridSwerve extends Command {
    private final Swerve s_Swerve;
    private final BlendedControl swerveBlend;

    public HybridSwerve(
        Swerve s_Swerve,
        BlendedControl inputs
    ) {
        this.s_Swerve = s_Swerve;
        this.swerveBlend = inputs;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        ChassisSpeeds driveOutputs = swerveBlend.solveChassisSpeedsInputs(s_Swerve.getHeading());
        s_Swerve.driveChassisSpeeds(driveOutputs, true);
    }
}