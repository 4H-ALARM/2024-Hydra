package frc.lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.config.krakenTalonConstants;

public final class CtreConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CtreConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Gear Ratio and Wrapping Config */
        // swerveAngleFXConfig.Feedback.SensorToMechanismRatio = krakenTalonConstants.Swerve.angleGearRatio;
        // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimit = 20;
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = krakenTalonConstants.Swerve.anglePid.kP;
        swerveAngleFXConfig.Slot0.kI = krakenTalonConstants.Swerve.anglePid.kI;
        swerveAngleFXConfig.Slot0.kD = krakenTalonConstants.Swerve.anglePid.kD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.NeutralMode = krakenTalonConstants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = krakenTalonConstants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = 50;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;


        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = krakenTalonConstants.Swerve.drivePid.kP;
        swerveDriveFXConfig.Slot0.kI = krakenTalonConstants.Swerve.drivePid.kI;
        swerveDriveFXConfig.Slot0.kD = krakenTalonConstants.Swerve.drivePid.kD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = krakenTalonConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = krakenTalonConstants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = krakenTalonConstants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = krakenTalonConstants.Swerve.closedLoopRamp;
    }
}