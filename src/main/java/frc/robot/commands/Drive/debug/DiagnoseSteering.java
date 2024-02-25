package frc.robot.commands.Drive.debug;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class DiagnoseSteering extends SequentialCommandGroup {
    public DiagnoseSteering(Swerve swerve){
        super(
                new TestSwerveSteerMotors(swerve, 0, 0.5).withTimeout(1.0),
                new TestSwerveSteerMotors(swerve, 1, 0.5).withTimeout(1.0),
                new TestSwerveSteerMotors(swerve, 2, 0.5).withTimeout(1.0),
                new TestSwerveSteerMotors(swerve, 3, 0.5).withTimeout(1.0)
//                new TestSwerveSteerMotors(swerve, 0, Units.degreesToRadians(90.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 0, Units.degreesToRadians(180.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 0, Units.degreesToRadians(270.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 0, Units.degreesToRadians(360.0)).withTimeout(1.0),
//
//                new TestSwerveSteerMotors(swerve, 1, Units.degreesToRadians(0.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 1, Units.degreesToRadians(90.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 1, Units.degreesToRadians(180.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 1, Units.degreesToRadians(270.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 1, Units.degreesToRadians(360.0)).withTimeout(1.0),
//
//                new TestSwerveSteerMotors(swerve, 2, Units.degreesToRadians(0.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 2, Units.degreesToRadians(90.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 2, Units.degreesToRadians(180.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 2, Units.degreesToRadians(270.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 2, Units.degreesToRadians(360.0)).withTimeout(1.0),
//
//                new TestSwerveSteerMotors(swerve, 3, Units.degreesToRadians(0.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 3, Units.degreesToRadians(90.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 3, Units.degreesToRadians(180.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 3, Units.degreesToRadians(270.0)).withTimeout(1.0),
//                new TestSwerveSteerMotors(swerve, 3, Units.degreesToRadians(360.0)).withTimeout(1.0)
                );
    }
}
