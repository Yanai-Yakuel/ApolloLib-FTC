package comp3_code_folder.org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "yanai")
public class Yanai extends OpMode {
    private DcMotor motor;
    private Servo servo;

    private double pos_servo1 = 0.5;
    private double pos_servo2 = 1.0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor1");
        servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            motor.setPower(1.0);
        } else {
            motor.setPower(0.0);
        }

        if (gamepad1.y) {
            servo.setPosition(pos_servo1);
        } else if (gamepad1.a) {
            servo.setPosition(pos_servo2);
        }
    }
}
