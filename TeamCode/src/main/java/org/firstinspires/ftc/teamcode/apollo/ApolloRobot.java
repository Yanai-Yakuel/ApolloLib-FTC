package org.firstinspires.ftc.teamcode.apollo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.apollo.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.apollo.subsystems.ShooterSubsystem;

/**
 * ApolloRobot - Lightweight container for the robot hardware.
 */
public class ApolloRobot {
    public final SampleMecanumDrive drive;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;

    public ApolloRobot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    public void update() {
        drive.update();
    }

    public void stop() {
        PoseStorage.currentPose = drive.getPoseEstimate();
        drive.setWeightedDrivePower(new Pose2d());
        shooter.setShooterPower(false);
        intake.stop();
    }
}
