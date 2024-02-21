package frc.lib;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class ModuleConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;

    public ModuleConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, boolean driveMotorInvert, boolean angleMotorInvert, SensorDirectionValue cancoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = cancoderInvert;
    }

    /** Swerve Drive Specialities */
    public static final class SDS {

        /** MK4i Module*/
        public static final class MK4i{
            /** Swerve Drive Specialties - MK4i Module (Double Falcon 500)*/
            public static ModuleConstants DoubleFalcon500(double driveGearRatio){
                double wheelDiameter = Units.inchesToMeters(4.0);

                // (150 / 7) : 1
                double angleGearRatio = ((150.0 / 7.0) / 1.0);
        
                double angleKP = 10.0;
                double angleKI = 0.0;
                double angleKD = 0.0;
        
                boolean driveMotorInvert = false;//InvertedValue.CounterClockwise_Positive;
                boolean angleMotorInvert = false;//InvertedValue.Clockwise_Positive;
                SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
                return new ModuleConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            public static ModuleConstants DoubleNeo(double driveGearRatio){
                double wheelDiameter = Units.inchesToMeters(4.0);
        
                // (150 / 7) : 1
                double angleGearRatio = ((150.0 / 7.0) / 1.0);
        
                double angleKP = 0.3;
                double angleKI = 0.0;
                double angleKD = 0.0;
        
                boolean driveMotorInvert = false;//InvertedValue.CounterClockwise_Positive;
                boolean angleMotorInvert = false;//InvertedValue.Clockwise_Positive;
                SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
                return new ModuleConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            /** Swerve Drive Specialties - MK4i Module (Kraken X60)*/
            public static ModuleConstants DoubleKrakenX60(double driveGearRatio){
                double wheelDiameter = Units.inchesToMeters(4.0);
        
                // (150 / 7) : 1
                double angleGearRatio = ((150.0 / 7.0) / 1.0);
        
                double angleKP = 1.0;
                double angleKI = 0.0;
                double angleKD = 0.0;

                boolean driveMotorInvert = false;//InvertedValue.CounterClockwise_Positive;
                boolean angleMotorInvert = false;//InvertedValue.Clockwise_Positive;
                SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
                return new ModuleConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            public static ModuleConstants KrakenFalcon(double driveGearRatio){
                double wheelDiameter = Units.inchesToMeters(4.0);

                // (150 / 7) : 1
                double angleGearRatio = ((150.0 / 7.0));

                double angleKP = 12.0;
                double angleKI = 0.0;
                double angleKD = 0.0;

                boolean driveMotorInvert = false;//InvertedValue.CounterClockwise_Positive;
                boolean angleMotorInvert = false;//InvertedValue.Clockwise_Positive;
                SensorDirectionValue cancoderInvert = SensorDirectionValue.Clockwise_Positive;
                return new ModuleConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            public static final class driveRatio {
                /** SDS MK4i - (8.14 : 1) */
                public static final double L1 = (8.14 / 1.0);

                public static final double L1and16t = (9.88/1.0);
            }
        }
    }
}

  