package comp3_code_folder.org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.PoseStorage;
import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
//@TeleOp(name = "COMP2_DO_NOT_TOUCH!", group = "drive")
public class comp2_code extends LinearOpMode {

    private DcMotor intakeMotor;
    private DcMotorEx shoot_u, shoot_d, transfer_motor;
    private Servo s_block, s_hood;
    private SampleMecanumDrive drive;
    private IMU imu; // ×”×©×ª×ž×© ×‘×©× ××—×™×“
    // PID Shooter
    public static double P = 100, I = 1.2, D = 3, F = 0;
    public static double SHOOT_TARGET_RPM = 2250;

    private double manualAngleOffset = 0;

    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 189;

    // --- HOLDING CONSTANTS ---
    public static double HOLD_KP = 0.04;
    public static double HOLD_KI = 0.006;
    public static double HOLD_KD = 0.15;
    public static double HOLD_KH = 0.5;
    public static double HOLD_MIN_PUSH = 0.07;

    private double xIntegral = 0, yIntegral = 0;

    // DRIVE
    public static double DRIVE_POWER = 0.8;
    public static double ROTATION_POWER = 0.8;

    // SERVO
    public static double B_OPEN = 0.556, B_CLOSE = 0.501; //0.501
    public static double hood_open = 0.47;

    enum ShootState { IDLE, OPEN_BLOCK, RUN_TRANSFER }
    ShootState currentShootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();

    private Pose2d targetPose = new Pose2d();
    private boolean holding = false;
    private boolean readyToShoot = false;
    private boolean lastGamepad1B = false;


    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        drive.setPoseEstimate(PoseStorage.currentPose);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        s_block = hardwareMap.get(Servo.class, "s_block");
        transfer_motor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        s_hood = hardwareMap.get(Servo.class, "s_hood");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        telemetry.addData("Clean IMU", "%.1fÂ°", Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        shoot_u.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDCoefficients();

        while (!isStarted() && !isStopRequested()) {
            displayPose();
            telemetry.update();
        }

        waitForStart();

        manualAngleOffset = drive.getPoseEstimate().getHeading() + Math.toRadians(90);

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            Pose2d pose = drive.getPoseEstimate();
            Pose2d poseVelocity = drive.getPoseVelocity();
            if (poseVelocity == null) poseVelocity = new Pose2d();

            s_hood.setPosition(hood_open);

            if (gamepad1.back) drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), 0));

            // INTAKE & TRANSFER
            handleIntake();

            // SHOOTER
            handleShooter(pose);

            if (gamepad1.a)
            {
                if (!holding)
                {
                    targetPose = pose;
                    resetIntegrals();
                    holding = true;
                }
            }
            else if (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                    Math.abs(gamepad1.left_stick_x) > 0.1 ||
                    Math.abs(gamepad1.right_stick_x) > 0.1)
            {
                holding = false;
            }

            if (holding)
            {
                executePD(pose, poseVelocity);
            } else
            {
                driveManual(pose);
            }
            sendTelemetry(pose);
        }
    }

    private void resetIntegrals() {
        xIntegral = 0;
        yIntegral = 0;
    }


    private void executePD(Pose2d pose, Pose2d poseVelocity) {
        double xError = targetPose.getX() - pose.getX();
        double yError = targetPose.getY() - pose.getY();
        double hError = Angle.normDelta(targetPose.getHeading() - pose.getHeading());

        if (Math.abs(xError) < 4) xIntegral = Math.max(-15, Math.min(15, xIntegral + xError));
        else xIntegral = 0;

        if (Math.abs(yError) < 4) yIntegral = Math.max(-15, Math.min(15, yIntegral + yError));
        else yIntegral = 0;

        double xCmd = (xError * HOLD_KP) + (xIntegral * HOLD_KI) - (poseVelocity.getX() * HOLD_KD);
        double yCmd = (yError * HOLD_KP) + (yIntegral * HOLD_KI) - (poseVelocity.getY() * HOLD_KD);
        double hCmd = (hError * HOLD_KH);

        if (Math.abs(xError) > 0.4) xCmd += Math.signum(xError) * HOLD_MIN_PUSH;
        if (Math.abs(yError) > 0.4) yCmd += Math.signum(yError) * HOLD_MIN_PUSH;

        double rotX = xCmd * Math.cos(-pose.getHeading()) - yCmd * Math.sin(-pose.getHeading());
        double rotY = xCmd * Math.sin(-pose.getHeading()) + yCmd * Math.cos(-pose.getHeading());

        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, hCmd));
    }

    private void driveManual(Pose2d pose) {
        double inputY = gamepad1.left_stick_y * DRIVE_POWER;
        double inputX = -gamepad1.left_stick_x * DRIVE_POWER;
        double rx = -gamepad1.right_stick_x * ROTATION_POWER;

        double botHeading = pose.getHeading() - manualAngleOffset;

        double rotX = inputX * Math.cos(-botHeading) - inputY * Math.sin(-botHeading);
        double rotY = inputX * Math.sin(-botHeading) + inputY * Math.cos(-botHeading);

        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rx));
    }
    private void handleIntake() {
        if (gamepad1.left_trigger > 0.1) {
            currentShootState = ShootState.IDLE;
            s_block.setPosition(B_CLOSE);
            // ×¤×œ×™×˜×” (Outtake)
            intakeMotor.setPower(0.8);
            transfer_motor.setPower(-0.3);
        } else if (gamepad1.right_trigger > 0.1) {
            // ××™×¡×•×£ (Intake)
            intakeMotor.setPower(-0.8);
            transfer_motor.setPower(0.8);
        } else if (currentShootState == ShootState.IDLE) {
            intakeMotor.setPower(0);
            transfer_motor.setPower(0);
        }
    }

    private void handleShooter(Pose2d pose) {
        if (gamepad1.b && !lastGamepad1B) {
            readyToShoot = !readyToShoot;
            if (readyToShoot) updatePIDCoefficients();
        }
        lastGamepad1B = gamepad1.b;

        double targetVel = (SHOOT_TARGET_RPM * TICKS_PER_REV) / 60.0;
        if (readyToShoot) {
            shoot_u.setVelocity(-targetVel);
            shoot_d.setVelocity(targetVel);
        } else {
            shoot_u.setVelocity(0);
            shoot_d.setVelocity(0);
        }

        double rpmU = (Math.abs(shoot_u.getVelocity()) * 60.0) / TICKS_PER_REV;
        double rpmD = (Math.abs(shoot_d.getVelocity()) * 60.0) / TICKS_PER_REV;
        boolean shooterReady = readyToShoot &&
                (Math.abs(SHOOT_TARGET_RPM - rpmU) < RPM_TOLERANCE) &&
                (Math.abs(SHOOT_TARGET_RPM - rpmD) < RPM_TOLERANCE);

        switch (currentShootState) {
            case IDLE:
                s_block.setPosition(B_CLOSE);
                if (gamepad1.dpad_up && shooterReady) {
                    shootTimer.reset();
                    s_block.setPosition(B_OPEN);
                    currentShootState = ShootState.OPEN_BLOCK;
                }
                break;
            case OPEN_BLOCK:
                if (shootTimer.milliseconds() > 300) {
                    transfer_motor.setPower(0.8);
                    intakeMotor.setPower(-0.8);
                    shootTimer.reset();
                    currentShootState = ShootState.RUN_TRANSFER;
                }
                break;
            case RUN_TRANSFER:
                if (shootTimer.milliseconds() > 1000) {
                    intakeMotor.setPower(0);
                    transfer_motor.setPower(0);
                    s_block.setPosition(B_CLOSE);
                    currentShootState = ShootState.IDLE;
                }
                break;
        }
    }


    private void updatePIDCoefficients() {
        shoot_u.setVelocityPIDFCoefficients(P, I, D, F);
        shoot_d.setVelocityPIDFCoefficients(P, I, D, F);
    }


    private void sendTelemetry(Pose2d pose) {
        telemetry.addData("Status", holding ? "HOLDING" : "MANUAL");
        telemetry.addData("X Error", targetPose.getX() - pose.getX());
        telemetry.addData("Y Error", targetPose.getY() - pose.getY());
        displayPose();
        telemetry.update();
    }



    private void displayPose() {
        Pose2d currentPose = drive.getPoseEstimate();
        telemetry.addData("Pose X", "%.2f", currentPose.getX());
        telemetry.addData("Pose Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.2fÂ°", Math.toDegrees(currentPose.getHeading()));
    }
}
