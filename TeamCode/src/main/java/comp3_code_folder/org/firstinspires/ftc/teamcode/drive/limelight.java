package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@TeleOp(name="Limelight TeleOp", group="Testing")
public class limelight extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // ×”×’×“×¨×ª ×”-IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // ×§×‘×œ×ª ×–×•×•×™×ª ×”-Yaw ×ž×”-IMU ×•×¢×“×›×•×Ÿ ×”×œ×™×ž×œ×™×™×˜ (×—×©×•×‘ ×œ-MT2)
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2(); // ×©×™×ž×•×© ×‘-MegaTag 2

            telemetry.addData("Target", "Detected");
            telemetry.addData("TX", llResult.getTx());
            telemetry.addData("TY", llResult.getTy());
            telemetry.addData("Botpose", botpose.toString());
        } else {
            telemetry.addData("Target", "Not Detected");
        }

        // ×¢×“×›×•×Ÿ ×”×˜×œ×ž×˜×¨×™×” ×‘×›×œ ×ž×¦×‘, ×’× ×× ××™×Ÿ ×ž×˜×¨×”
        telemetry.update();
    }
}
