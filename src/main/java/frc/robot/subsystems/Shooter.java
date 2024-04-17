package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterTopMotor;
    private final CANSparkFlex shooterBottomMotor;
    private PIDController vPidController;
    private final double velocitySetpoint;
    private double pidout;

    public final ShooterConfig config;
    public Shooter(ShooterConfig config) {
        this.config = config;
        shooterTopMotor = new CANSparkFlex(this.config.shooterTopMotor, CANSparkLowLevel.MotorType.kBrushless);

        shooterBottomMotor = new CANSparkFlex(this.config.shooterBottomMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottomMotor.follow(shooterTopMotor);

        vPidController = new PIDController(1, 0, 0);

        velocitySetpoint = 6000;


    }

    public void startShooter() {
<<<<<<< HEAD
        shooterTopMotor.set(0.55);
    }
    public void startAMPShooter() {
        shooterTopMotor.set(0.45);
=======
        shooterTopMotor.set(pidout);
>>>>>>> 8f94ce5 (s)
    }

    public void passNoteShooter() {
        shooterTopMotor.set(0.45);
    }

    public void stopShooter() {
        shooterTopMotor.stopMotor();
    }
    public void stopShoot() {
        shooterTopMotor.stopMotor();
    }

    public void sendBackShooter() {
        shooterTopMotor.set(-0.05);
    }

    public boolean isRevved() {
        return shooterTopMotor.getEncoder().getVelocity() > (velocitySetpoint - 200) && shooterTopMotor.getEncoder().getVelocity() < (velocitySetpoint + 200);
    }


    public boolean isShooterStopped() {
        if (shooterTopMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        pidout = vPidController.calculate(shooterTopMotor.getEncoder().getVelocity(), velocitySetpoint);
    }


}
