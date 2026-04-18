package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name = "Robot_Oriented_Drive", group = "drive")
public class RobotOrientedDrive extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intake;
    private DcMotor transfer_motor;

    @Override
    public void runOpMode() {

        frontleft  = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft   = hardwareMap.get(DcMotor.class, "backleft");
        backright  = hardwareMap.get(DcMotor.class, "backright");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        transfer_motor = hardwareMap.get(DcMotor.class, "transfer_motor");

        // ×¦×“ ×™×ž×™×Ÿ ×”×¤×•×š (×ž×§×× ×•× ×¡×˜× ×“×¨×˜×™)
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVE -----
            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(bl),
                                    Math.max(Math.abs(fr), Math.abs(br)))));

            frontleft.setPower(fl / max);
            backleft.setPower(bl / max);
            frontright.setPower(fr / max);
            backright.setPower(br / max);

            // ----- INTAKE + TRANSFER -----

            double leftTrigger  = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            if (leftTrigger > 0) {
                // ×©× ×™×”× ×¢×•×‘×“×™× ××‘×œ ×”×¤×•×š ××—×“ ×ž×”×©× ×™
                intake.setPower(leftTrigger);
                transfer_motor.setPower(-leftTrigger);
            }
            else if (rightTrigger > 0) {
                // intake ×‘×œ×‘×“ ×‘×›×™×•×•×Ÿ ×”×”×¤×•×š
                intake.setPower(-rightTrigger);
                transfer_motor.setPower(0);
            }
            else {
                intake.setPower(0);
                transfer_motor.setPower(0);
            }


            telemetry.addData("Intake", intake.getPower());
            telemetry.addData("Transfer", transfer_motor.getPower());
            telemetry.update();
        }
    }
}

