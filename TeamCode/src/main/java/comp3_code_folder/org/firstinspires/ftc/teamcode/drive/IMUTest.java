package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@TeleOp(group = "IMU Test")
public class IMUTest extends LinearOpMode {
    private IMU imu;   // ××• BNO055IMU, ×ª×œ×•×™ ×‘××™×–×” ××™× ×˜×¨××§×¦×™×” ××ª× ×ž×©×ª×ž×©×™×

    @Override
    public void runOpMode() {
        // ×× ×ž×©×ª×ž×©×™× ×‘Ö¾Universal IMU (×ž×•×“×¨× ×™)
        imu = hardwareMap.get(IMU.class, "imu");   // ×©× ×”×§×•×ž×¤×•× × ×˜ ×‘×§×•× ×¤×™×’

        // ××ª×—×•×œ (×× × ×“×¨×©)
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        telemetry.addData(">", "IMU initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addLine("=== IMU  TEST ===");
            telemetry.addData("IMU Yaw [rad]", "%.3f", yaw);
            telemetry.addData("IMU Yaw [deg]", "%.1f", Math.toDegrees(yaw));
            telemetry.update();
            sleep(50);
        }
    }
}

