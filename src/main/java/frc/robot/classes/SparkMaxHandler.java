package frc.robot.classes;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import frc.lib.config.PidConfig;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;

public class SparkMaxHandler {
    
    private final CANSparkMax sparkMax;

    public SparkMaxHandler(int CanID, CANSparkLowLevel.MotorType type, int CurrentLimit) {
        this.sparkMax = new CANSparkMax(CanID, type);
        this.sparkMax.restoreFactoryDefaults();
        this.sparkMax.clearFaults();
        this.sparkMax.setSmartCurrentLimit(CurrentLimit);
        this.sparkMax.setCANTimeout(0);
    }

    public SparkMaxHandler(int CanID) {
        this(
            CanID, 
            CANSparkLowLevel.MotorType.kBrushless,
            80
            );
        
    }

    public SparkMaxHandler(int CanID, CANSparkMax motorToFollow){
        this(
            CanID,
            CANSparkLowLevel.MotorType.kBrushless,
            80
        );
        this.sparkMax.follow(motorToFollow);
    }

    public SparkMaxHandler(int CanID, AbsoluteEncoder encoder) {
        this(
            CanID,
            CANSparkLowLevel.MotorType.kBrushless,
            80
        );
    }

    public double getInbuiltPosition() {
        return this.sparkMax.getEncoder().getPosition();
    }

    public AbsoluteEncoder getAbsoluteEncoder() {
        return sparkMax.getAbsoluteEncoder();
    }

    public SparkPIDController getPidController() {
        SparkPIDController sparkPID = sparkMax.getPIDController();
        sparkPID.setP(1);
        sparkPID.setI(0);
        sparkPID.setD(0);
        sparkPID.setFF(0);
        return sparkPID;
    }

    public SparkPIDController getPidController(PidConfig pidConfig) {
        SparkPIDController sparkPID = sparkMax.getPIDController();
        sparkPID.setP(pidConfig.kP);
        sparkPID.setI(pidConfig.kI);
        sparkPID.setD(pidConfig.kD);
        sparkPID.setFF(pidConfig.kF);
        return sparkPID;
    }

    public SparkPIDController getPIDController(PidConfig pidConfig, AbsoluteEncoder absoluteEncoder) {
        SparkPIDController sparkPID = sparkMax.getPIDController();
        sparkPID.setP(pidConfig.kP);
        sparkPID.setI(pidConfig.kI);
        sparkPID.setD(pidConfig.kD);
        sparkPID.setFF(pidConfig.kF);
        sparkPID.setFeedbackDevice(absoluteEncoder);
        return sparkPID;
    }

    public SparkPIDController getPidController(PidConfig pidConfig, boolean usePluggedEncoder) {
        SparkPIDController sparkPID = sparkMax.getPIDController();
        sparkPID.setP(pidConfig.kP);
        sparkPID.setI(pidConfig.kI);
        sparkPID.setD(pidConfig.kD);
        sparkPID.setFF(pidConfig.kF);
        return sparkPID;
    }



    

}
