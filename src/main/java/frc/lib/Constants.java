package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.config.*;

public final class Constants {

    public static IntakeConfig intakeConfig = new IntakeConfig(50, -0.8);
    public static ArmConfig armConfig = new ArmConfig(51, 52, new PidConfig(1.0, 0.0, 0.0), 0.51, 0.45, 0.661, 1, 163.32+180, new DigitalInput(2));
    public static ShooterConfig shooterConfig = new ShooterConfig(53, 54, -0.2, 10000);
    public static IndexerConfig indexerConfig = new IndexerConfig(55, -0.5, 0.15);
    public static LightConfig lightConfig = new LightConfig(0, 1, 1.5, 0);
    public static ColorSensorConfig colorSensorConfig = new ColorSensorConfig(2047);
    public static ClimberConfig climberConfig = new ClimberConfig(59, 58, 1.0, -1.0, new DigitalInput(1), new DigitalInput(6));
    public static VisionConfig visionConfig = new VisionConfig("limelight-intake", "limelight-shoot");

    public static SwerveModuleConfig mod0frontleftConfig = new SwerveModuleConfig(11, false, 12, false, 13, Rotation2d.fromRotations(0.95));
    public static SwerveModuleConfig mod1frontrightConfig = new SwerveModuleConfig(21, false, 22, false, 23, Rotation2d.fromRotations(-0.08));
    public static SwerveModuleConfig mod2backleftConfig = new SwerveModuleConfig(31, false, 32, false,33, Rotation2d.fromRotations(0.12));//
    public static SwerveModuleConfig mod3backrightConfig = new SwerveModuleConfig(41, false, 42, false, 43, Rotation2d.fromRotations(0.19));

    public static enum AutonomousOptions {
        TWO_NOTE_CENTER("TwoNoteCenter"),
        SHOOT_NOTE("ShootNote", true), // isDefault=true
        SHOOT_NOTE_MOVEBACK("ShootNoteMoveBack"),
        THREE_NOTES_RIGHT("ThreeNotesRight"),
        THREE_NOTES_LEFT("ThreeNoteLeft"),
        FOUR_NOTES("FourNoteAuto"),
        SEEK_NOTE("SEEK NOTE"),
        SEEK_PICKUP_NOTE("SEEK_PICKUP_NOTE"),
        SEEK_SPEAKER("SEEK_SPEAKER"),
        DEBUG_DRIVE("DEBUG DRIVE"),
        SHUFFLE("SHUFFLE"),
        SHOOT("SHOOT"),
        PICKUP("PICKUP"),
        ;


        private String name;
        private boolean isDefault;
        private AutonomousOptions(String name) {
            this(name, false);
        }

        private AutonomousOptions(String name, boolean isDefault) {
            this.name = name;
            this.isDefault = isDefault;
        }

        public static void registerChooser(SendableChooser<AutonomousOptions> chooser) {
            for (AutonomousOptions option : AutonomousOptions.values()) {
                chooser.addOption(option.name, option);
                if (option.isDefault) {
                    chooser.setDefaultOption(option.name, option);
                }
            }
        }
    }
}
