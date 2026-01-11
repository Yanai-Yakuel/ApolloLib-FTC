package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name = "Field Centric")
public class FieldCentricDrive extends OpMode {  // שם המחלקה תואם לשם הקובץ

    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private BNO055IMU imu;

    private boolean fieldCentricMode = true;
    private long lastToggleTime = 0;
    private long lastResetTime = 0;

    @Override
    public void init() {
        // הגדרת חומרה - אותם שמות מהקוד שלך
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");

        // הגדרת כיוונים (בדיוק כמו שלך)
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // הגדרות מנועים
        for (DcMotor motor : new DcMotor[]{backLeft, backRight, frontLeft, frontRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // הגדרת IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        telemetry.addData("Status", "Initialized - IMU Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        // החלפת מצב נסיעה (B) - בדיוק כמו שלך
        if (gamepad1.b && (currentTime - lastToggleTime > 300)) {
            fieldCentricMode = !fieldCentricMode;
            lastToggleTime = currentTime;
        }

        // איפוס זווית (START) - בדיוק כמו שלך
        if (gamepad1.start && (currentTime - lastResetTime > 500)) {
            imu.initialize(new BNO055IMU.Parameters());
            lastResetTime = currentTime;
        }

        // הגדרת מהירות - בדיוק כמו שלך
        double driveSpeed = gamepad1.a ? 0.4 : 0.8;

        // קלט מהשלט - בדיוק כמו שלך
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // קבלת הזווית מה-IMU במקום Pinpoint
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = angles.firstAngle;

        double rotX, rotY;
        if (fieldCentricMode) {
            // אותה מתמטיקה של Field Centric שלך
            rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        } else {
            rotX = strafe;
            rotY = forward;
        }

        // חישוב כוחות גולמיים למנועי Mecanum - בדיוק כמו שלך
        double flPower = rotY + rotX + rotate;
        double blPower = rotY - rotX + rotate;
        double frPower = rotY - rotX - rotate;
        double brPower = rotY + rotX - rotate;

        // נרמול מתוקן - תיקנתי את הסוגריים
        double max = Math.max(
                Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))
        );
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        // הפעלת הכוח - בדיוק כמו שלך
        frontLeft.setPower(flPower * driveSpeed);
        backLeft.setPower(blPower * driveSpeed);
        frontRight.setPower(frPower * driveSpeed);
        backRight.setPower(brPower * driveSpeed);

        // טלמטריה - בדיוק כמו שלך
        telemetry.addData("Mode", fieldCentricMode ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Heading (Deg)", Math.toDegrees(heading));
        telemetry.addData("Speed Multiplier", driveSpeed);
        telemetry.addLine("B: Toggle Mode | START: Reset Heading");
        telemetry.addData("FL", "%.2f", flPower);
        telemetry.addData("FR", "%.2f", frPower);
        telemetry.addData("BL", "%.2f", blPower);
        telemetry.addData("BR", "%.2f", brPower);

        telemetry.update();
    }
}
