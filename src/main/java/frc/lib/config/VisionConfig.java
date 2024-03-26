package frc.lib.config;

public class VisionConfig {
    public final String intakeLimelightName;
    public final String shootLimelightName;
    public final double offset;

    public VisionConfig(String intake, String shoot, double reversibleOffset) {
        this.intakeLimelightName = intake;
        this.shootLimelightName = shoot;
        this.offset = reversibleOffset;
    }

}
