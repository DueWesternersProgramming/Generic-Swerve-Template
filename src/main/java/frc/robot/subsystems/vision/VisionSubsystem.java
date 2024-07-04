package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;
import java.util.NoSuchElementException;

import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {

    public static Camera[] cameras = new Camera[2];
    public static CameraSim[] cameraSims = new CameraSim[2];
    private String[] cameraNames = { "backLeftCamera", "backRightCamera" };
    public static VisionSystemSim visionSim;

    public VisionSubsystem() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

            if (RobotBase.isSimulation()) {
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(CowboyUtils.aprilTagFieldLayout);

                visionSim.clearCameras();

                for (int i = 0; i < cameraNames.length; i++) {
                    cameraSims[i] = new CameraSim(cameraNames[i], VisionConstants.CAMERA_POSITIONS[i]);
                    visionSim.addCamera(cameraSims[i].photonCameraSim, VisionConstants.CAMERA_POSITIONS[i]);
                }

            } else {
                // Create as many camera instances as you have in the array cameraNames
                for (int i = 0; i < cameraNames.length; i++) {
                    cameras[i] = new Camera(cameraNames[i], VisionConstants.CAMERA_POSITIONS[i]);
                }
            }
        }

    }

    public static Pose2d getVisionPose(int i) throws NoSuchElementException {
        if (RobotBase.isSimulation()) {
            return cameraSims[i].getEstimatedGlobalPose(DriveSubsystem.getPose().orElseThrow())
                    .orElseThrow().estimatedPose.toPose2d();
        } else {
            return cameras[i].getEstimatedGlobalPose(DriveSubsystem.getPose().orElseThrow()).orElseThrow().estimatedPose
                    .toPose2d();
        }

    }

    public static int getLengthOfCameraList() {
        return cameras.length;
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            if (RobotBase.isSimulation()) {
                visionSim.update(DriveSubsystem.getPose().orElseThrow());
            }
        }
    }
}