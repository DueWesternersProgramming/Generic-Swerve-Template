package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;

public class CameraSim {
    private SimCameraProperties cameraProp;
    public PhotonCameraSim photonCameraSim;

    public CameraSim(PhotonCamera camera) {
        cameraProp = new SimCameraProperties();
        setCameraProperties();
        photonCameraSim = new PhotonCameraSim(camera, cameraProp);

    }

    private void setCameraProperties() {
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70)); // Make these constants
    }
}
