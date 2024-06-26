package frc.robot.classes.Limelight;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Constants;
import frc.robot.classes.Limelight.LimelightHelpers.LimelightResults;
import frc.robot.classes.Limelight.LimelightHelpers;


public class LimelightController {
    private LimelightResults llresults;

    private final String llName;




    public LimelightController(String name) {
        llName = name;
        this.llresults = LimelightHelpers.getLatestResults(llName);
        LimelightHelpers.setLEDMode_ForceOff(llName);
    }

    public void setOn() {
        LimelightHelpers.setLEDMode_ForceOn(llName);
    }

    public double getBasicRotOffset() {
        return LimelightHelpers.getTX(llName);
    }

    public LimelightResults getResults() {
        this.llresults = LimelightHelpers.getLatestResults(llName);
        return this.llresults;
    }

    public double getYawToSpeaker() {
        Pose3d pose = LimelightHelpers.getBotPose3d_TargetSpace(llName);
        double txToSpeaker = LimelightHelpers.getTX(llName);
        return txToSpeaker/20;
    }


    public double distanceToSpeaker() {
        double ty = LimelightHelpers.getTY(llName);
        return ty;
    }

    public double tagsSeen() {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
        return measurement.tagCount;
    }


    public double getYawToNote() {
        double limelightDegrees = LimelightHelpers.getTX(llName);
        return limelightDegrees / 27.0;
    }



    






}

//class LimelightController {
//
//    NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
//    NetworkTableEntry m_tx = m_table.getEntry("tx");
//    NetworkTableEntry m_ty = m_table.getEntry("ty");
//    NetworkTableEntry m_ta = m_table.getEntry("ta");
//    NetworkTableEntry m_tid = m_table.getEntry("tid");
//    NetworkTableEntry m_ledMode = m_table.getEntry("ledMode");
//    NetworkTableEntry m_pipeLine = m_table.getEntry("pipeline");
//    NetworkTableEntry m_llpython = m_table.getEntry("llpython");
//
//    private double m_llpythonReturn[];
//    private boolean m_lightOn = false;
//
//    private double m_currentPipeline = Constants.VisionParameters.k_aprilTagPipeline;
//
//    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
//    int numAprilTags = llresults.results.targets_Fiducials.length;
//
//    /**
//     * Creates a new vision.
//     */
//    public LimelightController() {
//
//        off();
//
//    }
////    public void periodic() {
////
////
////
////        double ledState = m_ledMode.getDouble(-1);
////        if (Constants.Debug.VisionON) {
////            SmartDashboard.putNumber("ledMode", ledState);
////        }
////
////
////        m_pipeLine.setNumber(m_currentPipeline);
////        if (Constants.Debug.VisionON) {
////            SmartDashboard.putNumber("Pipe", m_currentPipeline);
////        }
////
////
////    }
//
//    public void off() {
//        m_ledMode.setNumber(Constants.VisionParameters.k_lightOff);
//        m_lightOn = false;
//    }
//
//    public void on() {
//        m_ledMode.setNumber(Constants.VisionParameters.k_lightOn);
//        m_lightOn = true;
//    }
//
//    public boolean targetFound() {
//        boolean found = false;
//        double id = m_tid.getDouble(0.0);
//        if (id != -1) {
//            if (Math.abs(m_tx.getDouble(10)) <= Constants.VisionParameters.k_xTargetBounds) {
//                found = true;
//            }
//        }
//        return (found);
//    }
//
////    public double foundTargetID() {
////        if (targetFound()) {
////            return m_tid.getDouble();
////        }
////    }
//
//    public double getTx() {
//        return m_tx.getDouble(0);
//    }
//
//    public double get() {
//        return m_tx.getDouble(0);
//    }
//
//    /**
//     * Gets the vision returns from the pipeline selected
//     *
//     * @param pipe - the pipeline to read from
//     * @return - array of vision data, tx, ty and ta
//     */
//    public double[] findObject() {
//        double targetInfo[] = new double[3];
//
//        targetInfo[0] = m_tx.getDouble(0.0);
//        targetInfo[1] = m_ty.getDouble(0.0);
//        targetInfo[2] = m_ta.getDouble(0.0);
//
//        return (targetInfo);
//    }
//
//    public double getPipeline() {
//        return (m_currentPipeline);
//    }
//}