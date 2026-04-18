package comp3_code_folder.org.firstinspires.ftc.teamcode.autonomous;

import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.DriveConstants;
import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.PoseStorage;
import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Random;

@Autonomous(name = "autonomous_FAR_BLUE", group = "drive")
public class autonomous_far_blue extends LinearOpMode {
    private SampleMecanumDrive drive;
    private DcMotor intakeMotor, transfer_motor;
    private DcMotorEx shoot_u, shoot_d;
    private Servo s_block, s_hood;

    public static double P = 100, I = 1.2, D = 3, F = 0;
    public static double SHOOT_TARGET_RPM = 3050;
    private final double SHOOT_TARGET_VEL = (SHOOT_TARGET_RPM * 28.0) / 60.0;
    public static double B_OPEN = 0.556, B_CLOSE = 0.501;
    public static double hood_open = 0.42;

    private ElapsedTime shootTimer = new ElapsedTime();

    Pose2d startPose = new Pose2d(-64, 15, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        transfer_motor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        s_block = hardwareMap.get(Servo.class, "s_block");
        s_hood = hardwareMap.get(Servo.class, "s_hood");

        shoot_u.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);

        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        shoot_u.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot_d.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        drive.setPoseEstimate(startPose);

        String[] teamMembers = {"yanai", "amit", "nir", "yuri", "shvadi", "uri", "ya'ari", "tamir", "yogev", "neta", "benya", "peleg"};
        String selectedMember = teamMembers[new Random().nextInt(teamMembers.length)];

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Ready for", selectedMember);
            updateTelemetryPose();
        }

        waitForStart();
        if (isStopRequested()) return;

        s_block.setPosition(B_CLOSE);
        s_hood.setPosition(hood_open);

        start_shooter();

        sleep(4000);

            // ×™×¨×™ 1
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-61.02844, 18.175, Math.toRadians(25)))
                .build());

        shoot();

            /// ×›× ×™×¡×” ×œ××™×¡×•×£
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-32, 30.93, Math.toRadians(90)))
                .build());

        intakeMotor.setPower(-0.9);
        transfer_motor.setPower(0.8);
        start_shooter();


        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-34, 63.45, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build());

        intakeMotor.setPower(0);
        transfer_motor.setPower(0);


        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-70, 25, Math.toRadians(20)))
                .build());
        shoot();

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-36.7224, 30.93, Math.toRadians(90)))
                .build());
        stopRobot();
    }
    private void start_shooter()
    {
        shoot_u.setVelocity(SHOOT_TARGET_VEL);
        shoot_d.setVelocity(SHOOT_TARGET_VEL);
    }
    private void shoot() {

        shootTimer.reset();

        s_block.setPosition(B_OPEN);

        while (shootTimer.milliseconds() < 300 && opModeIsActive()) {

            drive.update();

        }
        transfer_motor.setPower(0.7);
        intakeMotor.setPower(-0.7);
        shootTimer.reset();
        while (shootTimer.milliseconds() < 1000 && opModeIsActive()) {

            drive.update();

        }
        shootTimer.reset();
        while (shootTimer.milliseconds() < 300 && opModeIsActive()) {
            drive.update();

        }

        intakeMotor.setPower(0);
        transfer_motor.setPower(0);
        s_block.setPosition(B_CLOSE);
    }

    private void stopRobot() {
        shoot_u.setVelocity(0);
        shoot_d.setVelocity(0);
        PoseStorage.currentPose = drive.getPoseEstimate();
        telemetry.addLine("Autonomous Complete");
        updateTelemetryPose();
    }

    private void updateTelemetryPose() {
        Pose2d currentPose = drive.getPoseEstimate();
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.2fÂ°", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}
