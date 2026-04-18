package comp3_code_folder.org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "benchtest")
public class teleop_code extends OpMode {
    private DcMotor motor;
    private Servo servo;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        // servo.setPosition(0.0);
    }

    @Override
    public void loop() {
        double dir = -gamepad1.left_stick_y;
        motor.setPower(dir);
        telemetry.update();
    }
}
