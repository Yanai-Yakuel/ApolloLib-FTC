package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;

@Config
@TeleOp(name = "Competition_2026", group = "drive")
public class CompetitionDrive extends LinearOpMode {

    // Hardware
    private DcMotor intakeMotor;
    private DcMotorEx shoot_u, shoot_d, transfer_motor;
    private Servo s_block, s_hood;
    private SampleMecanumDrive drive;
    private Limelight3A limelight;

    // Target Pose
    public static Pose2d TARGET_POSE_RED = new Pose2d(-26.7, 24.0175, Math.toRadians(135));
    public static final double M_TO_IN = 39.37;

    // Align PID
    public static double HOLD_KP = 0.045;
    public static double HOLD_KI = 0.003;
    public static double HOLD_KD = 0.18;
    public static double HOLD_KH = 0.9;
    public static double HOLD_MIN_PUSH = 0.05;

    private double xIntegral = 0;
    private double yIntegral = 0;

    public static double DRIVE_POWER = 0.8;
    public static double ROTATION_POWER = 0.8;

    public static double B_OPEN = 0.556;
    public static double B_CLOSE = 0.501;
    public static double hood_open = 0.47;

    // Shooter PID
    public static double SHOOT_TARGET_RPM = 2250;
    public static final double TICKS_PER_REV = 28.0;

    enum ShootState { IDLE, OPEN_BLOCK, RUN_TRANSFER }
    ShootState currentShootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean limelightAlign = false;
    private boolean readyToShoot = false;
    private boolean lastGamepad1B = false;

    // X Edge Trigger
    private boolean lastGamepad1X = false;

    // Manual offset
    private double manualAngleOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        transfer_motor = hardwareMap.get(DcMotorEx.class, "transfer_motor");

        s_block = hardwareMap.get(Servo.class, "s_block");
        s_hood = hardwareMap.get(Servo.class, "s_hood");

        shoot_u.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);

        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        manualAngleOffset = drive.getPoseEstimate().getHeading() + Math.toRadians(90);

        while (opModeIsActive() && !isStopRequested()) {

            drive.update();
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d poseVelocity = drive.getPoseVelocity();
            if (poseVelocity == null) poseVelocity = new Pose2d();

            s_hood.setPosition(hood_open);

            boolean tagVisible = false;

            // Limelight update only during Align
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    tagVisible = true;
                    if (limelightAlign) updateBotPose(result);
                }
            }

            // X button: reset heading to 90° (Edge trigger)
            if (gamepad1.x && !lastGamepad1X) {
                drive.setPoseEstimate(new Pose2d(
                        currentPose.getX(),
                        currentPose.getY(),
                        Math.toRadians(90)
                ));
            }
            lastGamepad1X = gamepad1.x;

            // Y button: start Limelight Align
            if (gamepad1.y && !limelightAlign) {
                limelightAlign = true;
                resetIntegrals();
            }

            // Any stick input cancels align
            if (Math.abs(gamepad1.left_stick_y) > 0.15 ||
                    Math.abs(gamepad1.left_stick_x) > 0.15 ||
                    Math.abs(gamepad1.right_stick_x) > 0.15) {
                limelightAlign = false;
                // Update manual offset to current heading for smooth Field Oriented
                manualAngleOffset = currentPose.getHeading() - Math.toRadians(90);
            }

            // Execute PD or Manual drive
            if (limelightAlign) {
                executePD(currentPose, poseVelocity, TARGET_POSE_RED);
            } else {
                driveManual(currentPose);
            }

            handleIntake();
            handleShooter();
            sendTelemetry(currentPose, tagVisible, limelightAlign);
        }

        if (limelight != null) limelight.stop();
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
    private void executePD(Pose2d currentPose, Pose2d vel, Pose2d target) {

        double xErr = target.getX() - currentPose.getX();
        double yErr = target.getY() - currentPose.getY();
        double hErr = Angle.normDelta(target.getHeading() - currentPose.getHeading());

        double distance = Math.hypot(xErr, yErr);

        if (distance < 0.5 && Math.abs(hErr) < Math.toRadians(2)) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            limelightAlign = false;
            manualAngleOffset = currentPose.getHeading() - Math.toRadians(90);
            xIntegral = 0;
            yIntegral = 0;
            return;
        }

        // אינטגרלים רק אם רחוק מספיק
        if (Math.abs(xErr) < 5) xIntegral = Math.max(-10, Math.min(10, xIntegral + xErr * 0.02));
        else xIntegral = 0;

        if (Math.abs(yErr) < 5) yIntegral = Math.max(-10, Math.min(10, yIntegral + yErr * 0.02));
        else yIntegral = 0;

        // פקודת PD
        double xCmd = xErr * HOLD_KP + xIntegral * HOLD_KI - vel.getX() * (HOLD_KD * 0.5); // KD נמוך יותר
        double yCmd = yErr * HOLD_KP + yIntegral * HOLD_KI - vel.getY() * (HOLD_KD * 0.5);

        // Ramp-down לפי מרחק
        double maxSpeed = Math.min(0.7, distance * 0.1); // קרוב -> יותר איטי

        // Field Oriented
        double heading = currentPose.getHeading();
        double rotX = xCmd * Math.cos(-heading) - yCmd * Math.sin(-heading);
        double rotY = xCmd * Math.sin(-heading) + yCmd * Math.cos(-heading);

        // הגבלת מהירות לפי ramp-down
        rotX = Math.max(-maxSpeed, Math.min(maxSpeed, rotX));
        rotY = Math.max(-maxSpeed, Math.min(maxSpeed, rotY));

        // סיבוב
        double rotPower = Math.max(-0.5, Math.min(0.5, hErr * HOLD_KH)); // מוגבל כדי למנוע קפיצות
        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rotPower));
    }

    private void driveManual(Pose2d pose) {
        double inputY = -gamepad1.left_stick_y * DRIVE_POWER;
        double inputX = gamepad1.left_stick_x * DRIVE_POWER;
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
        if (gamepad1.b && !lastGamepad1B) readyToShoot = !readyToShoot;
        lastGamepad1B = gamepad1.b;

        double targetVel = (SHOOT_TARGET_RPM * TICKS_PER_REV) / 60.0;
        if (readyToShoot) {
            shoot_u.setVelocity(-targetVel);
            shoot_d.setVelocity(targetVel);
        } else {
            shoot_u.setVelocity(0);
            shoot_d.setVelocity(0);
        }

        switch (currentShootState) {
            case IDLE:
                s_block.setPosition(B_CLOSE);
                if (gamepad1.dpad_up && readyToShoot) {
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

    private void sendTelemetry(Pose2d pose, boolean tagVisible, boolean aligning) {
        telemetry.addData("Mode", aligning ? "ALIGNING" : "MANUAL");
        telemetry.addData("Source", tagVisible ? "BOTPOSE" : "ODOMETRY");
        telemetry.addData("Target", "(-26.7, 24.0)");
        telemetry.addData("Current", "(%.1f, %.1f)", pose.getX(), pose.getY());
        if (aligning) {
            telemetry.addData("X Error", "%.1f", TARGET_POSE_RED.getX() - pose.getX());
            telemetry.addData("Y Error", "%.1f", TARGET_POSE_RED.getY() - pose.getY());
        }
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
