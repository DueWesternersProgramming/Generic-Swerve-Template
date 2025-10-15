package frc.robot.subsystems.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.utils.TimestampedPose;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNavIO io;
    QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

    public QuestNavSubsystem(QuestNavIO io) {
        this.io = io;
    }

    public void setRobotPose(Pose2d pose) {
        System.out.println("Setting robot pose");
        io.setRobotPose(pose);
    }

    public Pose2d getRobotPose() {
        return io.getCorrectedPose();
    }

    public Boolean isConnected() {
        return io.isConnected();
    }

    public Command resetPoseYaw(Rotation2d yaw) {
        return new InstantCommand(() -> {
            Pose2d currentPose = io.getCorrectedPose();
            Pose2d newPose = new Pose2d(currentPose.getTranslation(), yaw);
            io.setRobotPose(newPose);
        }, this);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("QuestNavSubsystem", inputs);

        // Do filtering here in the future...
        Boolean isPoseWithinTolerance = true;

        if (inputs.correctedPose.getTranslation().getDistance(RobotState.robotPose.getTranslation()) > Units
                .inchesToMeters(10)) {
            isPoseWithinTolerance = true;
        }
        Logger.recordOutput("QuestNavSubsystem/isPoseInTolerance", isPoseWithinTolerance);

        // if (DriverStation.isEnabled() && RobotState.isQuestNavPoseReset) {
        RobotState.offerQuestMeasurement(new TimestampedPose(getRobotPose(),
                inputs.timestamp));
        // }
    }

}
