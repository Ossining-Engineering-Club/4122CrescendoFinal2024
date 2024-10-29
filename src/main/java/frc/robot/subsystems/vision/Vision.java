package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class Vision extends SubsystemBase {
    public static record CameraConfig(String name, Transform3d robotToCam) {}
    public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;

    public Vision(CameraConfig... configs) {
        cameras = new PhotonCamera[configs.length];
        estimators = new PhotonPoseEstimator[configs.length];

        for (int i = 0; i < configs.length; i++) {
            PhotonCamera camera = new PhotonCamera(configs[i].name());
            PhotonPoseEstimator estimator =
                new PhotonPoseEstimator(
                    VisionConstants.TAG_LAYOUT,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera,
                    configs[i].robotToCam());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            cameras[i] = camera;
            estimators[i] = estimator;
        }
    }

    public PoseEstimate[] getEstimatedGlobalPoses() {
        List<PoseEstimate> estimates = new ArrayList<>();
        List<Pose3d> detectedTagPoses = new ArrayList<>();
        for (int i = 0; i < estimators.length; i++) {
            var result = cameras[i].getLatestResult();
            var optionalEstimate = estimators[i].update(result);

            // adding detected tags to list to be logged
            for (PhotonTrackedTarget tag : result.getTargets()) {
                VisionConstants.TAG_LAYOUT.getTagPose(tag.getFiducialId())
                    .ifPresent((tagPose) -> detectedTagPoses.add(tagPose)); // if there's a pose, add it to the list
            }

            if (optionalEstimate.isPresent()) {
                var estimate = optionalEstimate.get();

                // don't use if estimate is outside the field
                if (!(estimate.estimatedPose.getX() > 0.0 && estimate.estimatedPose.getX() <= constants.kFieldLengthMeters &&
                    estimate.estimatedPose.getY() > 0.0 && estimate.estimatedPose.getY() <= constants.kFieldWidthMeters)) continue;
                // don't use if estimate is too high, or too tilted
                if (Math.abs(estimate.estimatedPose.getZ()) > VisionConstants.MAX_HEIGHT) continue;
                if (Math.abs(estimate.estimatedPose.getRotation().getX()) > VisionConstants.MAX_ANGLE) continue;
                if (Math.abs(estimate.estimatedPose.getRotation().getY()) > VisionConstants.MAX_ANGLE) continue;

                estimates.add(
                    new PoseEstimate(estimate, getEstimationStdDevs(estimate.estimatedPose.toPose2d(), result)));
            }
        }
        // logging detected tags
        Logger.recordOutput("Detected Tag Poses", detectedTagPoses.toArray(Pose3d[]::new));
        
        return estimates.toArray(PoseEstimate[]::new);
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
        var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        var targets = pipelineResult.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            // if nonexistent tag, ignore
            var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;

        avgDist /= numTags;

        // decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
        // increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 7) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        if (VisionConstants.IGNORE_YAW) estStdDevs.set(2, 0, Double.MAX_VALUE);

        return estStdDevs;
    }
}
