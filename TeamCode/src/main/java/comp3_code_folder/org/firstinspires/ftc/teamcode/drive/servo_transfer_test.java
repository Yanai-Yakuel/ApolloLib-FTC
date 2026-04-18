package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "servo_transfer_test", group = "test")
public class servo_transfer_test extends LinearOpMode {

    private Servo s_transfer;

    private DcMotor intake;

    double pos = 0.5;
    //    double pos = 0;
    final double STEP = 0.002;

    @Override
    public void runOpMode() throws InterruptedException {

        s_transfer = hardwareMap.get(Servo.class, "s_block");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // ×× ×¦×¨×™×š ×œ×”×¤×•×š ×›×™×•×•×Ÿ:
        // s_transfer.setDirection(Servo.Direction.REVERSE);

        s_transfer.setPosition(pos);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                sleep(100);
                pos += STEP;
            }

            if (gamepad1.dpad_down) {
                sleep(100);
                pos -= STEP;
            }

            pos = Math.max(0, Math.min(1, pos));

            s_transfer.setPosition(pos);   // ðŸ”¥ ×–×” ×ž×” ×©×—×¡×¨



            // ---- QUICK POSITIONS ----


            if (gamepad1.a) {intake.setPower(-0.8);}
            if (gamepad1.b){intake.setPower(0);}

            // Clamp
            pos = Math.max(0, Math.min(1, pos));

            telemetry.addLine("SERVO UNIT TEST");
            telemetry.addData("Dpad Up", "Increase");
            telemetry.addData("Dpad Down", "Decrease");
            telemetry.addData("Servo Pos", "%.4f", pos);
            telemetry.update();
        }
    }
}

