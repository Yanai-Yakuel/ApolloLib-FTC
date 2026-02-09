package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Competition_2026", group = "drive")
public class CompetitionDrive extends LinearOpMode {

    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx shoot_u, shoot_d;
    private Servo s_transfer, s_hood;
    private SampleMecanumDrive drive;

    // === PID לכיול ===
    public static double P = 30, I = 2, D = 0, F = 13.5;
    public static double SHOOT_TARGET_RPM = 1400;
    private final double SHOOT_TARGET_VEL = (SHOOT_TARGET_RPM * 28.0) / 60.0;

    // === PD Holding ===
    public static double HOLD_KP = 0.035;
    public static double HOLD_KD = 0.14;
    public static double HOLD_KH = 0.45;

    // === DRIVE ===
    public static double DRIVE_POWER = 0.8;
    public static double ROTATION_POWER = 0.65;

    // === SERVO POSITIONS ===
    public static double CLOSE_POS = 0.7;
    public static double OPEN_POS = 0.92;
    public static double S_HOOD_SHOOT = 1.0;
    public static double S_HOOD_R = 0.21;

    public static boolean updatePID = false;

    // === State Management ===
    private enum ShootState { IDLE, PUSH, RETRACT, FILL }
    private ShootState shootState = ShootState.IDLE;
    private int shotCount = 0;
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean readyToShoot = false;
    private boolean lastGamepad1B = false;
    private boolean lastDpadLeft = false; // ✅ המשתנה נמצא כאן למעלה

    private Pose2d targetPose = new Pose2d();
    private boolean holding = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(5.4, 32.35, Math.toRadians(90)));

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        s_hood = hardwareMap.get(Servo.class, "s_hood");
        s_transfer = hardwareMap.get(Servo.class, "s_transfer");

        shoot_u.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_u.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        shoot_d.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        s_transfer.setPosition(OPEN_POS);
        s_hood.setPosition(S_HOOD_R);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Pose2d pose = drive.getPoseEstimate();
            Pose2d poseVelocity = drive.getPoseVelocity();
            if (poseVelocity == null) poseVelocity = new Pose2d();

            TelemetryPacket packet = new TelemetryPacket();

            // === 1. INTAKE ===
            double intakePower = 0;
            if (gamepad1.left_trigger > 0.1) {
                intakePower = 0.8;
            } else if (shootState == ShootState.FILL) {
                intakePower = -0.8;
            } else if (gamepad1.right_trigger > 0.1) {
                intakePower = -0.8;
            }
            intakeMotor.setPower(intakePower);

            if (gamepad1.dpad_down) {
                transferMotor.setPower(-1.0);
            }

            // === 2. SHOOTER (Toggle with B) ===
            if (gamepad1.b && !lastGamepad1B) {
                readyToShoot = !readyToShoot;
            }
            lastGamepad1B = gamepad1.b;

            if (gamepad1.x) readyToShoot = false;

            if (readyToShoot) {
                shoot_u.setVelocity(SHOOT_TARGET_VEL);
                shoot_d.setVelocity(SHOOT_TARGET_VEL);
                s_hood.setPosition(S_HOOD_SHOOT);
                if (!gamepad1.dpad_down) transferMotor.setPower(1.0);
            } else {
                shoot_u.setVelocity(0);
                shoot_d.setVelocity(0);
                s_hood.setPosition(S_HOOD_R);
                if (!gamepad1.dpad_down) transferMotor.setPower(0);
            }

            if (updatePID) {
                shoot_u.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
                shoot_d.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            }

            // === 3. SHOOTING STATE MACHINE ===
            if (gamepad1.dpad_up && shootState == ShootState.IDLE && readyToShoot) {
                shootState = ShootState.PUSH;
                shotCount = 0;
                shootTimer.reset();
            }

            switch (shootState) {
                case PUSH:
                    s_transfer.setPosition(CLOSE_POS);
                    if (shootTimer.milliseconds() > 900) { shootState = ShootState.RETRACT; shootTimer.reset(); }
                    break;
                case RETRACT:
                    s_transfer.setPosition(OPEN_POS);
                    if (shootTimer.milliseconds() > 900) { shootState = ShootState.FILL; shootTimer.reset(); }
                    break;
                case FILL:
                    if (shootTimer.milliseconds() > 900) {
                        shotCount++;
                        if (shotCount < 1) shootState = ShootState.PUSH;
                        else shootState = ShootState.IDLE;
                        shootTimer.reset();
                    } break;
                case IDLE: s_transfer.setPosition(OPEN_POS); break;
            }


            // === 4. DRIVE & RESET ===

// בתוך ה-While, במקום מה שיש עכשיו:
            if (gamepad1.dpad_left && !lastDpadLeft) {
                // נסה לשנות ל-90 מעלות (Math.toRadians(90))
                // אם זה עדיין הפוך, נסה 0 או 180.
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), Math.toRadians(90)));
                gamepad1.rumble(200); // פידבק שהצליח
            }
            lastDpadLeft = gamepad1.dpad_left;
            if (gamepad1.a) {
                targetPose = new Pose2d(23.4, 22, Math.toRadians(45));
                holding = true;
            } else if (gamepad1.y) {
                targetPose = new Pose2d(47.1, 10.7, Math.toRadians(75));
                holding = true;
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                holding = false;
            }

            if (holding) {
                executePD(pose, poseVelocity);
            } else {
                double ly = gamepad1.left_stick_x * DRIVE_POWER;
                double lx = gamepad1.left_stick_y * DRIVE_POWER;
                double rx = -gamepad1.right_stick_x * ROTATION_POWER;

                double botHeading = pose.getHeading();
                double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
                double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);
                drive.setWeightedDrivePower(new Pose2d(rotY, -rotX, rx));
            }

            // === Telemetry & Dashboard ===
            packet.put("shoot_u_vel", shoot_u.getVelocity());
            packet.put("shoot_d_vel", shoot_d.getVelocity());
            packet.fieldOverlay()
                    .setStroke("cyan").strokeCircle(targetPose.getX(), targetPose.getY(), 6)
                    .setFill("green").fillCircle(pose.getX(), pose.getY(), 3);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Ready", readyToShoot);
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }

    private void executePD(Pose2d pose, Pose2d poseVelocity) {
        double xError = targetPose.getX() - pose.getX();
        double yError = targetPose.getY() - pose.getY();
        double hError = Angle.normDelta(targetPose.getHeading() - pose.getHeading());

        double xCmd = (xError * HOLD_KP) - (poseVelocity.getX() * HOLD_KD);
        double yCmd = (yError * HOLD_KP) - (poseVelocity.getY() * HOLD_KD);
        double hCmd = (hError * HOLD_KH);

        double rotX = xCmd * Math.cos(-pose.getHeading()) - yCmd * Math.sin(-pose.getHeading());
        double rotY = xCmd * Math.sin(-pose.getHeading()) + yCmd * Math.cos(-pose.getHeading());
        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, hCmd));
    }
}
