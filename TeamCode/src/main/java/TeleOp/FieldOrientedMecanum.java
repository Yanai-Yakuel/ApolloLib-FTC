package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="Field Oriented Mecanum", group="Examples")
public class FieldOrientedMecanum extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;

    private boolean fieldMode = false; // toggle field-oriented

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void loop() {
        // toggle mode
        if (gamepad1.a) fieldMode = false;     // robot-oriented
        if (gamepad1.b) fieldMode = true;      // field-oriented

        double stickX = -gamepad1.left_stick_x;
        double stickY = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;

        double x = stickX;
        double y = stickY;

        if (fieldMode) {
            double heading = getHeading();
            double cosA = Math.cos(-heading);
            double sinA = Math.sin(-heading);

            double fieldX = stickX * cosA - stickY * sinA;
            double fieldY = stickX * sinA + stickY * cosA;

            x = fieldX;
            y = fieldY;
        }

        double fl = y + x + rotation;
        double fr = y - x - rotation;
        double bl = y - x + rotation;
        double br = y + x - rotation;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        telemetry.addData("Mode", fieldMode ? "Field" : "Robot");
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
