package frc.robot.classes;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;;

public class KrakenX60MotorHandler {
    private final TalonFX kraken;
    private TalonFXConfiguration config;

    public KrakenX60MotorHandler(int CanID, String Canbus, double StatorCurrentLimit, boolean enableLimit) {
        this.kraken = new TalonFX(CanID, Canbus);
        this.config = new TalonFXConfiguration();
        this.config.CurrentLimits.StatorCurrentLimit = StatorCurrentLimit;
        this.config.CurrentLimits.StatorCurrentLimitEnable = enableLimit;
        this.kraken.getConfigurator().apply(this.config);
        this.kraken.setPosition(0.0);
    }

    public KrakenX60MotorHandler(int CanID, double StatorCurrentLimit) {
        this(CanID, null, StatorCurrentLimit, true);
    }

    public TalonFX getMotor() {
        return kraken;
    }

    

}
