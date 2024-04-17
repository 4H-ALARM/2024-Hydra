package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.IntakeConfig;
import frc.robot.classes.ColorSensorController;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    public final IntakeConfig config;
    public final ColorSensorController colorSensor;

    public Intake(IntakeConfig config, ColorSensorController colorSensorController) {
        this.config = config;
        intakeMotor = new CANSparkMax(this.config.intakeMotorId, CANSparkLowLevel.MotorType.kBrushless);
        colorSensor = colorSensorController;
    }



    public void pickUpNote() {
        intakeMotor.set(this.config.intakeMotorSpeed);
    }

    public void rejectNote() {
        intakeMotor.set(-this.config.intakeMotorSpeed);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public boolean isIntakeStopped() {
        if (intakeMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }

    public boolean finishedIntaking() {
        return colorSensor.isSeen();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isIntaked", colorSensor.isSeen());
    }

}
