package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Axon_Unit_Test", group = "test")
public class AxonUnitTest extends LinearOpMode {

    private Servo axon;
    private DcMotor intake;
    private DcMotor transferMotor;

    private DcMotor shoot_d;
    private DcMotor shoot_u;

    double pos = 0.52;
    final double STEP = 0.002;

    boolean prevUp = false;
    boolean prevDown = false;

    @Override
    public void runOpMode() throws InterruptedException {

        axon = hardwareMap.get(Servo.class, "s_block");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");

        shoot_d = hardwareMap.get(DcMotor.class, "shoot_d");
        shoot_u = hardwareMap.get(DcMotor.class, "shoot_u");

        // ×¢×¦×™×¨×” ×—×“×” ×œ×©×•×˜×¨
        shoot_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot_u.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ×›×™×•×•×Ÿ ×”×¤×•×š ×œ×¤×™ SonarQube ×ª×§× ×™
        shoot_u.setDirection(DcMotorSimple.Direction.REVERSE);

        axon.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {

            // ----- SERVO CONTROL -----
            boolean currentUp = gamepad1.dpad_up;
            boolean currentDown = gamepad1.dpad_down;

            if (currentUp && !prevUp) {
                pos += STEP;
            }

            if (currentDown && !prevDown) {
                pos -= STEP;
            }

            prevUp = currentUp;
            prevDown = currentDown;

            pos = Math.max(0, Math.min(1, pos));
            axon.setPosition(pos);

            // ----- INTAKE -----
            if (gamepad1.a) {
                intake.setPower(-0.8);
            } else if (gamepad1.b) {
                intake.setPower(0);
            }

            // ----- TRANSFER -----
            if (gamepad1.x) {
                transferMotor.setPower(1.0);
            }

            if (gamepad1.y) {
                transferMotor.setPower(0);
            }

            // ----- SHOOTER -----
            if (gamepad1.dpad_up) {
                shoot_d.setPower(1.0);
                shoot_u.setPower(1.0);  // ×›×™×•×•×Ÿ ×›×‘×¨ ×”×¤×•×š ×œ×ž×¢×œ×”
            }

            if (gamepad1.dpad_down) {
                shoot_d.setPower(0);
                shoot_u.setPower(0);
            }

            telemetry.addLine("AXON UNIT TEST");
            telemetry.addData("Servo Pos", "%.4f", pos);
            telemetry.addData("Shooter Power", shoot_d.getPower());
            telemetry.update();
        }
    }
}

