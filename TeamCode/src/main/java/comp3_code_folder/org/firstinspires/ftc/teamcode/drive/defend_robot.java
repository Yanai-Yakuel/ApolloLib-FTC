package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//@TeleOp(name = "DEFENSE_ROBOT", group = "Drive")
public class defend_robot extends LinearOpMode {


    DcMotor frontLeft, frontRight, backLeft, backRight;


    @Override
    public void runOpMode() {


        // Map motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);






        waitForStart();


        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double frontLeftPower = forward + strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backLeftPower = forward + strafe - turn;
            double backRightPower = forward - strafe + turn;

            frontLeft.setPower(Math.min(1, frontLeftPower));
            frontRight.setPower(Math.min(1, frontRightPower));
            backLeft.setPower(Math.min(1, backLeftPower));
            backRight.setPower(Math.min(1, backRightPower));

            telemetry.addData("turn", turn);
            telemetry.update();
        }
    }
}


