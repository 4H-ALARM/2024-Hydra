package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.VisionConfig;
import frc.lib.util.Tags;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.classes.RollingAverage;


public class Vision {

    private final LimelightController shootLimelight;
    private final LimelightController intakeLimelight;
    private final Tags tags;

    private final PIDController shootPID;
    private final PIDController intakePID;
    private final RollingAverage intakeAverage = new RollingAverage(5);
    private final RollingAverage shootAverage = new RollingAverage(5);
    private final RollingAverage shootDistAverage = new RollingAverage(5);

    private double aimRotationPower;
    private double angleToShootAngle;

    private final VisionConfig config;

    private double shootSetPoint = 0;

    public Vision(VisionConfig visionConfig, DriverStation.Alliance alliance) {
        this.config = visionConfig;
        this.shootLimelight = new LimelightController(config.shootLimelightName);
        this.intakeLimelight = new LimelightController(config.intakeLimelightName);
        this.shootPID = new PIDController(1.25, 0.01, 0.2);
//        this.intakePID = new PIDController(0.01, 0, 0);
//        this.intakePID = new CustomPid(0.25, 0.2, 0);
        this.intakePID = new PIDController(2.0, 0.01, .20);
        this.aimRotationPower = 0.0;
        this.angleToShootAngle = 0.0;
        this.tags = new Tags();
        if (alliance == DriverStation.Alliance.Blue) {
            shootLimelight.switchShooterBluePipline();
        } else {
            shootLimelight.switchShooterRedPipline();
        }
    }

    /**
     * The output of a PID loop, from -1.0 to 1.0, describing
     * how hard the swerve should rotate to aim at the target
     */
    public double getNoteAimRotationPower() {
        return aimRotationPower;
    }

    public double getAngleToShootAngle() {
        return angleToShootAngle;
    }

    public void periodic() {
        shootSetPoint = 0;
        if (shootLimelight.twoTagsSeen() < 2) {
            if (shootLimelight.tagsSeen() == this.tags.Blue_Off_Center_Tag()) {
                shootSetPoint = config.offset;
            }
            if (shootLimelight.tagsSeen() == this.tags.Red_Off_Center_Tag()) {
                shootSetPoint = -config.offset;
            }
        }
        intakeAverage.addInput(intakeLimelight.getYawToNote());
<<<<<<< HEAD
        shootAverage.addInput(shootLimelight.getYawToSpeaker());
        aimRotationPower = intakePID.calculate(intakeAverage.getOutput(), 0);
        angleToShootAngle = shootPID.calculate(shootAverage.getOutput(), 0);
        SmartDashboard.putNumber("intakePID", aimRotationPower);
        SmartDashboard.putNumber("shootPID", aimRotationPower);
=======
        shootAverage.addInput(shootLimelight.getYawToShootTarget());
        angleToNote = intakePID.calculate(intakeAverage.getOutput(), 0);
        angleToShootAngle = shootPID.calculate(shootAverage.getOutput(), shootSetPoint);
        SmartDashboard.putNumber("intakePID", angleToNote);
        SmartDashboard.putNumber("shootPID", angleToNote);
>>>>>>> f261a94 (offsets?)
    }
}