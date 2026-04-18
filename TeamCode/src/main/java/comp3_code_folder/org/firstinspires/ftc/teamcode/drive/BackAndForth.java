package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 40;
    private IMU imuClean;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        imuClean = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imuClean.initialize(parameters);



        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ×§×“×™×ž×”
            Trajectory forward = drive.trajectoryBuilder(startPose)
                    .forward(DISTANCE)
                    .build();

            drive.followTrajectory(forward);

            // ××—×•×¨×”
            Trajectory backward = drive.trajectoryBuilder(forward.end())
                    .back(DISTANCE)
                    .build();

            drive.followTrajectory(backward);
        }
    }
}
