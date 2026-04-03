package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

// CAMERA YAY

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera processingCamera;
    public PhotonPipelineResult result;

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
}
