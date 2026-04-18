package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "SYSTEMS_CHECK_FINAL_2026", group = "test")
public class systems_check extends LinearOpMode {

    // Hardware
    private DcMotor intakeMotor;
    private DcMotorEx shoot_u, shoot_d, transfer_motor;
    private Servo s_block, s_hood;

    // Constants from TeleOp
    private final double B_OPEN = 0.556, B_CLOSE = 0.501;
    private final double HOOD_OPEN = 0.47;
    private final double TARGET_RPM = 2250.0;
    private final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 1. INITIALIZATION ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transfer_motor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        s_block = hardwareMap.get(Servo.class, "s_block");
        s_hood = hardwareMap.get(Servo.class, "s_hood");

        // Sync directions with CompetitionDrive
        shoot_u.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("âœ” Hardware Mapped");
        telemetry.addLine("READY - Press Play to start diagnostics");
        telemetry.update();

        waitForStart();

        boolean shooterToggle = false;
        boolean lastB = false;
        boolean hoodToggle = false;
        boolean lastRB = false;

        while (opModeIsActive()) {
            // --- 2. SHOOTER TEST (B Toggle) ---
            if (gamepad1.b && !lastB) shooterToggle = !shooterToggle;
            lastB = gamepad1.b;

            double targetTicksPerSec = (TARGET_RPM * TICKS_PER_REV) / 60.0;
            if (shooterToggle) {
                shoot_u.setVelocity(-targetTicksPerSec);
                shoot_d.setVelocity(targetTicksPerSec);
            } else {
                shoot_u.setPower(0);
                shoot_d.setPower(0);
            }

            // --- 3. INTAKE & TRANSFER (Triggers) ---
            // RT: Intake (Full system), LT: Outtake (Safety check)
            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(-0.8);
                transfer_motor.setPower(0.8);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(0.8);
                transfer_motor.setPower(-0.5);
            } else {
                intakeMotor.setPower(0);
                // X key for independent Transfer test
                transfer_motor.setPower(gamepad1.x ? 0.8 : 0);
            }

            s_block.setPosition(gamepad1.y ? B_OPEN : B_CLOSE);

            // RB: Hood Toggle (To test range of motion)
            if (gamepad1.right_bumper && !lastRB) hoodToggle = !hoodToggle;
            lastRB = gamepad1.right_bumper;
            s_hood.setPosition(hoodToggle ? HOOD_OPEN : 0.2); // 0.2 is a safe closed value

            telemetry.addLine("=== âš™ï¸ SYSTEM DIAGNOSTICS ===");

            // Shooter Data
            double rpm_u = (Math.abs(shoot_u.getVelocity()) * 60.0) / TICKS_PER_REV;
            double rpm_d = (Math.abs(shoot_d.getVelocity()) * 60.0) / TICKS_PER_REV;
            telemetry.addData("Shooter", shooterToggle ? "ON" : "OFF");
            telemetry.addData("Upper RPM", "%.0f / %.0f", rpm_u, TARGET_RPM);
            telemetry.addData("Lower RPM", "%.0f / %.0f", rpm_d, TARGET_RPM);

            telemetry.addLine("--- ðŸ› ï¸ Servos ---");
            telemetry.addData("Block (Y)", gamepad1.y ? "OPEN" : "CLOSED");
            telemetry.addData("Hood Pos", "%.3f", s_hood.getPosition());

            telemetry.addLine("--- âš¡ Motors ---");
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Transfer Power", transfer_motor.getPower());


            telemetry.update();
        }
    }
}
