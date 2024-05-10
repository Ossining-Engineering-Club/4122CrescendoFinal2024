package frc.robot.subsystems;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Photon extends SubsystemBase {
    private final PhotonCamera m_camera;
    private final Field2d m_field = new Field2d();
    private final PhotonPoseEstimator m_poseEstimator;
    private final AtomicReference<EstimatedRobotPose> m_atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    private final AtomicReference<EstimatedRobotPose> m_displayPose = new AtomicReference<EstimatedRobotPose>();
    private double readingtime;

    public Photon(String cameraName, Transform3d robotToCamera) {
    
        m_camera = new PhotonCamera(cameraName);
        var aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        m_poseEstimator = new PhotonPoseEstimator(
            aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCamera);
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void update() {
        
        var results = m_camera.getLatestResult();
        // checks if has targets
        if (results.hasTargets()
            // and either has 2 or more tags
            && (results.targets.size() > 1
                // or has a single tag with a pose ambiguity less than the threshold and a distance from the camera less than the threshold
                || (results.targets.get(0).getPoseAmbiguity() < constants.kAprilTagAmbiguityThreshold
                    && results.targets.get(0).getBestCameraToTarget().getTranslation().getDistance(new Translation3d(0, 0, 0)) < constants.kSingleTagMaxDistance))) { 
                        // update with results after filtering
                        m_poseEstimator.update(results).ifPresent(estimatedRobotPose -> {
                            var estimatedPose = estimatedRobotPose.estimatedPose;
                            // checks if pose is within the field
                            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= constants.kFieldLengthMeters &&
                                estimatedPose.getY() > 0.0 && estimatedPose.getY() <= constants.kFieldWidthMeters) {
                                    m_atomicEstimatedRobotPose.set(estimatedRobotPose);
                                    m_displayPose.set(estimatedRobotPose);
                            }
                        });
        }
        
    }

    public EstimatedRobotPose getLatestEstimatedPose() {
        return m_atomicEstimatedRobotPose.getAndSet(null);
    }
    public Pose2d getVisionPose(){
        if (m_displayPose.get() != null)
            return m_displayPose.get().estimatedPose.toPose2d();
        else
            return null;
    }
    
    @Override
    public void periodic() {
        if (m_displayPose.get() != null) {
            m_field.setRobotPose(m_displayPose.get().estimatedPose.toPose2d());
            SmartDashboard.putData("Filtered Photon Pose", m_field);
        }
    }
}