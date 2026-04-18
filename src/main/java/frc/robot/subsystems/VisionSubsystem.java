package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.targeting.PhotonTrackedTarget;

// CAMERA YAY

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera processingCamera;
    public PhotonPipelineResult result;
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, DriveConstants.kRobotCameraPosition);

    public VisionSubsystem() {
        processingCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        
    }

    @Override
    public void periodic() {
        result = processingCamera.getLatestResult();
    }

    public PhotonPipelineResult gResult() {
        return result;
    }

    public PhotonTrackedTarget getTrackedTarget() {
        return result.getBestTarget();
    }
}
