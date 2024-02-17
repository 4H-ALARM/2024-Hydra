package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class DiagnoseSwerve extends SequentialCommandGroup {
    public DiagnoseSwerve (Swerve swerve){
        super(
                new TestSwerve(swerve, 1.0,0.0,0).withTimeout(2.0),
                new TestSwerve(swerve, 0.0,1.0,0).withTimeout(2.0),
                new TestSwerve(swerve, -1.0,0.0,0).withTimeout(2.0),
                new TestSwerve(swerve, 0.0,-1.0,0).withTimeout(2.0)

        );
    }
}
