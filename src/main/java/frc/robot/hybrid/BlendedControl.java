package frc.robot.hybrid;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * BlendedControl is a class for blending multiple input sources to create a single output
 *
 * It supports 2 input types: ControlVector and ChassisSpeeds. The ControlVectors are first
 * blended to yield one ChassisSpeeds, then this ChassisSpeeds is blended with any additional
 * ChassisSpeeds such as those from the PathPlanner.
 */
public class BlendedControl {
    // ControlVector components to blend
    private final ArrayList<Supplier<ControlVector>> pValues = new ArrayList<>();
    private final ArrayList<Supplier<ControlVector>> tValues = new ArrayList<>();

    // ChassisSpeeds components to blend
    private final ArrayList<Supplier<ChassisSpeeds>> pSpeeds = new ArrayList<>();
    private final ArrayList<Supplier<ChassisSpeeds>> tSpeeds = new ArrayList<>();

    /**
     * Add a new ControlVector input component to the blended control by providing
     * @param pValue function to get latest pValue ControlVector for this component
     * @param tValue function to get latest tValue ControlVector for this component
     */
    public void addControlVectorComponent(Supplier<ControlVector> pValue, Supplier<ControlVector> tValue) {
        pValues.add(pValue);
        tValues.add(tValue);
    }

    /**
     * Add a new ChassisSpeeds input component to the blended control by providing the following:
     * @param pValue function to get latest pValue ChassisSpeed for this component
     * @param tValue function to get latest tValue ChassisSpeed for this component
     *
     * The tSpeed ChassisSpeed value acts as an influence vector whose components are multiplied
     * against the tSpeed vectors.
     */
    public void addChassisSpeedsComponent(Supplier<ChassisSpeeds> pSpeed, Supplier<ChassisSpeeds> tSpeed) {
        pSpeeds.add(pSpeed);
        tSpeeds.add(tSpeed);
    }

    /**
     * Multiply the components of all inputs by their corresponding weights, then sum them
     * @return A SwerveVector representing the blend of all inputs
     */
    public ControlVector solveControlVectorInputs() {
        double fX = 0; // Field X
        double fY = 0; // Field Y
        double rX = 0; // Robot X
        double rY = 0; // Robot Y
        double rot = 0; // Robot rotation
        double arm = 0; // Arm power

        for (int i = 0; i < pValues.size(); i++) {
            // Calculate weighted fieldX
            double pFx = pValues.get(i).get().swerveFieldX();
            double tFx = tValues.get(i).get().swerveFieldX();
            fX += pFx * tFx;

            // Calculate weighted fieldY
            double pFy = pValues.get(i).get().swerveFieldY();
            double tFy = tValues.get(i).get().swerveFieldY();
            fY += pFy * tFy;

            // Calculate weighted robotX
            double pRx = pValues.get(i).get().swerveRobotX();
            double tRx = tValues.get(i).get().swerveRobotX();
            rX += pRx * tRx;

            // Calculate weighted robotY
            double pRy = pValues.get(i).get().swerveRobotY();
            double tRy = tValues.get(i).get().swerveRobotY();
            rY += pRy * tRy;

            // Calculate weighted rotation
            double pRot = pValues.get(i).get().swerveRotation();
            double tRot = tValues.get(i).get().swerveRotation();
            rot += pRot * tRot;

            // Calculate weighted arm power
            double pArm = pValues.get(i).get().armPower();
            double tArm = tValues.get(i).get().armPower();
            arm += pArm * tArm;
        }

        return new ControlVector(fX, fY, rX, rY, rot, arm);
    }

    /**
     * 1. Solve the blending of all ControlVectors given to this blend
     * 
     */
    public ChassisSpeeds solveChassisSpeedsInputs(Rotation2d robotHeading) {
        ControlVector blendedControlVector = solveControlVectorInputs();
        ChassisSpeeds controlVectorChassisSpeeds = blendedControlVector.calculateChassisSpeeds(robotHeading);

        // Solve the blend of all input ChassisSpeeds
        double rX = controlVectorChassisSpeeds.vxMetersPerSecond; // Robot X
        double rY = controlVectorChassisSpeeds.vyMetersPerSecond; // Robot Y
        double rot = controlVectorChassisSpeeds.omegaRadiansPerSecond; // Robot rotation
        for (int i = 0; i < pSpeeds.size(); i++) {
            // Calculate weighted robotX
            double pRx = pSpeeds.get(i).get().vxMetersPerSecond;
            double tRx = tSpeeds.get(i).get().vxMetersPerSecond;
            rX += pRx * tRx;

            // Calculate weighted robotY
            double pRy = pSpeeds.get(i).get().vyMetersPerSecond;
            double tRy = tSpeeds.get(i).get().vyMetersPerSecond;
            rY += pRy * tRy;

            // Calculate weighted rotation
            double pRot = pSpeeds.get(i).get().omegaRadiansPerSecond;
            double tRot = tSpeeds.get(i).get().omegaRadiansPerSecond;
            rot += pRot * tRot;
        }

        // TODO: calculate blend of ChassisSpeeds
        return new ChassisSpeeds(rX, rY, rot);
    }
}
