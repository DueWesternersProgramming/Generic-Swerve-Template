package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;
import java.util.NoSuchElementException;

import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {

    public static Camera[] cameras = new Camera[2];
    public static CameraSim[] cameraSims = new CameraSim[2];
    private String[] cameraNames = { "frontLeftCamera", "frontRightCamera" };

    // TODO move this to
    // constants
    private Transform3d[] cameraPositions = { new Transform3d(0, 0, 0, new Rotation3d()),
            new Transform3d(0, 0, 0, new Rotation3d()) };

    public static VisionSystemSim visionSim;

    public VisionSubsystem() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

            // Create as many camera instances as you have in the array cameraNames
            for (int i = 0; i < cameraNames.length; i++) {
                cameras[i] = new Camera(cameraNames[i], cameraPositions[i]);
            }

            if (RobotBase.isSimulation()) {
                System.out.println("OH YEAH");
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(CowboyUtils.aprilTagFieldLayout);

                visionSim.clearCameras();

                for (int i = 0; i < cameraNames.length; i++) {
                    cameraSims[i] = new CameraSim(cameras[i].photonCamera);
                }

            }
        }

    }

    public static Pose2d getVisionPose(int i) throws NoSuchElementException {

        return cameras[i].getEstimatedGlobalPose(DriveSubsystem.getPose().orElseThrow()).orElseThrow().estimatedPose
                .toPose2d();

    }

    public static int getLengthOfCameraList() {
        return cameras.length;
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            // SmartDashboard.putBoolean("Has a tracked object:", cameras[0].hasResults());
            visionSim.update(DriveSubsystem.getPose().orElseThrow());
        }
    }
}