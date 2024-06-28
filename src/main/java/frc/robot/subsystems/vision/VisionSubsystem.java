package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.NoSuchElementException;

public class VisionSubsystem extends SubsystemBase {

    private static Camera[] cameras = new Camera[2];
    private String[] cameraNames = { "frontLeftCamera", "frontRightCamera" };

    public VisionSubsystem() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

            // Create as many camera instances as you have

            cameras[0] = new Camera(cameraNames[0], new Transform3d(0, 0, 0, new Rotation3d())); // Front Left
            cameras[0] = new Camera(cameraNames[0], new Transform3d(0, 0, 0, new Rotation3d())); // Front Right
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
            SmartDashboard.putBoolean("Has a tracked object:", cameras[0].hasResults());
        }
    }
}