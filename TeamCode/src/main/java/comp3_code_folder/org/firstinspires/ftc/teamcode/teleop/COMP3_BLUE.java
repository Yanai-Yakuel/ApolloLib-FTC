package comp3_code_folder.org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.PoseStorage;
import comp3_code_folder.org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "comp_BLUEðŸ”µ", group = "drive")
public class COMP3_BLUE extends LinearOpMode {

    // Hardware
    private DcMotor intakeMotor;
    private DcMotorEx shoot_u, shoot_d, transfer_motor;
    private Servo s_block, s_hood;
    private SampleMecanumDrive drive;
    private Limelight3A limelight;

    // Target Pose
    public static Pose2d TARGET_POSE_BLUE = new Pose2d(-17.05, -16.78, Math.toRadians(-135));
    public static Pose2d TARGET_POSE_ = new  Pose2d(-26.7, 24.0175, Math.toRadians(135));

    public static final double M_TO_IN = 39.37;

    // PID Drive
    public static double HOLD_KP = 0.045;
    public static double HOLD_KI = 0.003;
    public static double HOLD_KD = 0.18;
    public static double HOLD_KH = 1.2;

    private double xIntegral = 0;
    private double yIntegral = 0;
    private IMU imuClean;

    // PID Shooter
    public static double P = 100, I = 1.2, D = 3, F = 0;
    public static double SHOOT_TARGET_RPM = 2250;
    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 130;

    // Drive
    public static double DRIVE_POWER = 0.9;
    public static double ROTATION_POWER = 0.9;

    // Servo
    public static double B_OPEN = 0.556;
    public static double B_CLOSE = 0.501;
    public static double hood_open = 0.47;

    enum ShootState {
        IDLE, OPEN_BLOCK, RUN_TRANSFER
    }

    ShootState currentShootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean limelightAlign = false;
    private boolean readyToShoot = false;
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1back = false;
    private boolean lastGamepad1Y = false;

    private double manualAngleOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ðŸš€ ×—×™×‘×•×¨ ×œ-Dashboard - ×”×›×™ ×—×©×•×‘!
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        transfer_motor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        s_block = hardwareMap.get(Servo.class, "s_block");
        s_hood = hardwareMap.get(Servo.class, "s_hood");

        imuClean = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imuClean.initialize(parameters);

        telemetry.addData("Clean IMU", "%.1fÂ°", Math.toDegrees(
                imuClean.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.addData("Drive IMU", "%.1fÂ°", Math.toDegrees(
                drive.getRawExternalHeading()));
        telemetry.update();

        shoot_u.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDCoefficients();

        // Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(0);
                limelight.start();
            }
        } catch (Exception e) {
            limelight = null;
        }

        waitForStart();
        Pose2d currentPose = drive.getPoseEstimate();
        reset_pos(new Pose2d(currentPose.getX(), currentPose.getY(),Math.toRadians(90)));

        while (opModeIsActive() && !isStopRequested()) {

            drive.update();
            currentPose = drive.getPoseEstimate();
            Pose2d poseVelocity = drive.getPoseVelocity();
            if (poseVelocity == null)
                poseVelocity = new Pose2d();

            s_hood.setPosition(hood_open);

            // Limelight
            boolean tagVisible = false;
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    tagVisible = true;
                }
            }

            // X button: reset heading to 90Â°
            if (gamepad1.back && !lastGamepad1back) {
                reset_pos(new Pose2d(
                        currentPose.getX(),
                        currentPose.getY(),
                        Math.toRadians(90)));
            }
            lastGamepad1back = gamepad1.back;

            // Y button: align ×¨×§ ×× ×¨×•××™× ×ª×’
            if (gamepad1.y && !lastGamepad1Y) {
                if (limelight != null) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        updateBotPose(result);
                        limelightAlign = true;
                        resetIntegrals();
                    }
                }
            }
            lastGamepad1Y = gamepad1.y;

            // Any stick input cancels align
            if (Math.abs(gamepad1.left_stick_y) > 0.15 ||
                    Math.abs(gamepad1.left_stick_x) > 0.15 ||
                    Math.abs(gamepad1.right_stick_x) > 0.15) {
                limelightAlign = false;
            }

            if (limelightAlign) {
                if (!tagVisible) {
                    limelightAlign = false;
                    drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                } else {
                    executePID(currentPose, poseVelocity, TARGET_POSE_BLUE);
                }
            } else {
                driveManual();
            }

            handleIntake();
            handleShooter();
            sendTelemetry(currentPose, tagVisible, limelightAlign);
        }

        if (limelight != null)
            limelight.stop();
    }

    private void updateBotPose(LLResult result) {
        Pose3D botPose = result.getBotpose();
        if (botPose != null && botPose.getPosition() != null) {
            double x = botPose.getPosition().x * M_TO_IN;
            double y = botPose.getPosition().y * M_TO_IN;
            double yaw = Math.toRadians(botPose.getOrientation().getYaw());
            drive.setPoseEstimate(new Pose2d(x, y, yaw));
        }
    }

    private void resetIntegrals() {
        xIntegral = 0;
        yIntegral = 0;
    }

    private void updatePIDCoefficients() {
        shoot_u.setVelocityPIDFCoefficients(P, I, D, F);
        shoot_d.setVelocityPIDFCoefficients(P, I, D, F);
    }

    private void executePID(Pose2d currentPose, Pose2d vel, Pose2d target) {
        double xErr = target.getX() - currentPose.getX();
        double yErr = target.getY() - currentPose.getY();
        double hErr = Angle.normDelta(target.getHeading() - currentPose.getHeading());

        // ðŸ“Š × ×ª×•× ×™× ×œ-PID Drive Graph
        telemetry.addData("xError", xErr);
        telemetry.addData("yError", yErr);
        telemetry.addData("hError", Math.toDegrees(hErr));

        double distance = Math.hypot(xErr, yErr);

        if (distance < 0.1 && Math.abs(hErr) < Math.toRadians(1)) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            limelightAlign = false;
            xIntegral = 0;
            yIntegral = 0;
            return;
        }

        if (Math.abs(xErr) < 5)
            xIntegral = Math.max(-10, Math.min(10, xIntegral + xErr * 0.02));
        else
            xIntegral = 0;

        if (Math.abs(yErr) < 5)
            yIntegral = Math.max(-10, Math.min(10, yIntegral + yErr * 0.02));
        else
            yIntegral = 0;

        double xCmd = xErr * HOLD_KP + xIntegral * HOLD_KI - vel.getX() * (HOLD_KD * 0.5);
        double yCmd = yErr * HOLD_KP + yIntegral * HOLD_KI - vel.getY() * (HOLD_KD * 0.5);

        double maxSpeed = Math.min(0.8, distance * 0.15 + 0.2);

        double heading = currentPose.getHeading();
        double rotX = xCmd * Math.cos(-heading) - yCmd * Math.sin(-heading);
        double rotY = xCmd * Math.sin(-heading) + yCmd * Math.cos(-heading);

        rotX = Math.max(-maxSpeed, Math.min(maxSpeed, rotX));
        rotY = Math.max(-maxSpeed, Math.min(maxSpeed, rotY));

        double rotPower = Math.max(-0.5, Math.min(0.5, hErr * HOLD_KH));
        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rotPower));
    }

    private void driveManual() {
        Pose2d pose = drive.getPoseEstimate();
        double inputY = -gamepad1.left_stick_y * DRIVE_POWER;
        double inputX = gamepad1.left_stick_x * DRIVE_POWER;
        double rx = -gamepad1.right_stick_x * ROTATION_POWER;

        double botHeading = pose.getHeading() - manualAngleOffset;

        double rotX = inputX * Math.cos(-botHeading) - inputY * Math.sin(-botHeading);
        double rotY = inputX * Math.sin(-botHeading) + inputY * Math.cos(-botHeading);

        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rx));
    }

    private void reset_pos(Pose2d current_pos){
        drive.setPoseEstimate(current_pos);
    }

    private void handleIntake() {
        if (gamepad1.left_trigger > 0.1) {
            currentShootState = ShootState.IDLE;
            s_block.setPosition(B_CLOSE);
            intakeMotor.setPower(0.8);
            transfer_motor.setPower(-0.3);
        } else if (gamepad1.right_trigger > 0.1) {
            intakeMotor.setPower(-0.8);
            transfer_motor.setPower(0.8);
        } else if (currentShootState == ShootState.IDLE) {
            intakeMotor.setPower(0);
            transfer_motor.setPower(0);
        }
    }

    private void handleShooter() {
        if (gamepad1.b && !lastGamepad1B) {
            readyToShoot = !readyToShoot;
            if (readyToShoot)
                updatePIDCoefficients();
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

        // ðŸ“Š ×—×™×©×•×‘ RPM ×œ-Graph
        double rpmU = (Math.abs(shoot_u.getVelocity()) * 60.0) / TICKS_PER_REV;
        double rpmD = (Math.abs(shoot_d.getVelocity()) * 60.0) / TICKS_PER_REV;

        // ðŸš€ × ×ª×•× ×™× ×©×™×•×¤×™×¢×• ×‘-Graph ×©×œ RPM!
        telemetry.addData("rpmTarget", SHOOT_TARGET_RPM);
        telemetry.addData("rpmU", rpmU);
        telemetry.addData("rpmD", rpmD);
        telemetry.addData("rpmErrorU", SHOOT_TARGET_RPM - rpmU);
        telemetry.addData("rpmErrorD", SHOOT_TARGET_RPM - rpmD);

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
                    if (shootTimer.milliseconds() < 400) {
                        transfer_motor.setPower(-0.5);
                    } else {
                        transfer_motor.setPower(0.8);
                        shootTimer.reset();
                        currentShootState = ShootState.RUN_TRANSFER;
                    }
                    intakeMotor.setPower(-0.8);
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

    private void sendTelemetry(Pose2d pose, boolean tagVisible, boolean aligning ) {
        double cleanImu = imuClean.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double driveImu = drive.getRawExternalHeading();
        double poseHead = pose.getHeading();

        telemetry.addLine("=== IMU DEBUG ===");
        telemetry.addData("Clean IMU", "%.1fÂ°", Math.toDegrees(cleanImu));
        telemetry.addData("Drive IMU", "%.1fÂ°", Math.toDegrees(driveImu));

        telemetry.addLine("");
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");

        if (aligning) {
            telemetry.addData("X Error", "%.1f", TARGET_POSE_BLUE.getX() - pose.getX());
            telemetry.addData("Y Error", "%.1f", TARGET_POSE_BLUE.getY() - pose.getY());
            telemetry.addData("H Error", "%.1fÂ°",
                    Math.toDegrees(Angle.normDelta(TARGET_POSE_BLUE.getHeading() - pose.getHeading())));
        }
        telemetry.update();
    }
}

