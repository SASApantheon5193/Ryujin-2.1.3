package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.utilities.LimelightHelpers;

public class VisionHelper {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final ADIS16470_IMU gyro;
    private static final String LIMELIGHT_NAME = "limelight";

    private Pose2d lastVisionPose = new Pose2d();

    // Hardcoded toggle for MegaTag1 vs MegaTag2
    private static final boolean USE_MEGA_TAG2 = true;  // true = MegaTag2, false = MegaTag1

    public Pose2d getLastVisionPose() {
        return lastVisionPose;
    }

    public VisionHelper(SwerveDrivePoseEstimator poseEstimator, ADIS16470_IMU gyro) {
        this.poseEstimator = poseEstimator;
        this.gyro = gyro;
        SmartDashboard.putBoolean("/Vision/Enable Vision Updates", true);
    }

    public void update() {
        boolean visionEnabled = SmartDashboard.getBoolean("/Vision/Enable Vision Updates", true);
        boolean useMegaTag2 = USE_MEGA_TAG2;

        if (!visionEnabled) {
            logBoolean("Vision/Used Vision Update", false);
            return;
        }

        LimelightHelpers.SetRobotOrientation(
                LIMELIGHT_NAME,
                poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
                0, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        logString("Vision/MegaTag1 Pose", mt1.pose.toString());
        logString("Vision/MegaTag2 Pose", mt2.pose.toString());

        LimelightHelpers.PoseEstimate estimate = useMegaTag2 ? mt2 : mt1;

        if (estimate == null || estimate.tagCount == 0 || Math.abs(gyro.getRate()) > 360.0 || estimate.avgTagDist > 3.0) {
            logBoolean("Vision/Used Vision Update", false);
            return;
        }

        // Reject if vision pose is way too far from odometry (>1 meter jump)
        if (estimate.pose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) > 1.0) {
            logBoolean("Vision/Used Vision Update", false);
            logString("Vision/Reject Reason", "Pose Jump Too Large");
            return;
        }

        // Tuned trust for vision update to avoid shaking
        double xyStdDev = MathUtil.clamp(0.05 * estimate.avgTagDist, 0.3, 1.0);


        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
        lastVisionPose = estimate.pose;

        logDouble("Vision/Avg Tag Distance", estimate.avgTagDist);
        logBoolean("Vision/Used Vision Update", true);
    }

    private void logDouble(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    private void logBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    private void logString(String key, String value) {
        SmartDashboard.putString(key, value);
    }
}
