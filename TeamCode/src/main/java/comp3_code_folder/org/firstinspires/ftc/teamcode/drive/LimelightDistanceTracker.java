package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//@TeleOp(name="Limelight Distance Tracker", group="Testing")
public class LimelightDistanceTracker extends OpMode {

    private Limelight3A limelight;
    

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        telemetry.addData("Status", "Initialized - Ready to scan");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            java.util.List<FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                FiducialResult fiducial = fiducials.get(0);

                Pose3D targetPoseRobotSpace = fiducial.getTargetPoseRobotSpace(); // AprilTag ×‘×™×—×¡ ×œ×¨×•×‘×•×˜

                if (targetPoseRobotSpace != null) {
                    // ×‘Ö¾Robot/Target Space, ×”×™×—×™×“×•×ª ×”×Ÿ ×ž×˜×¨×™×
                    double forwardDist = targetPoseRobotSpace.getPosition().z * 100.0;
                    double sideDist    = targetPoseRobotSpace.getPosition().x * 100.0;
                    double verticalDist= targetPoseRobotSpace.getPosition().y * 100.0;

                    telemetry.addData("Target ID", fiducial.getFiducialId());
                    telemetry.addLine("--- DISTANCE (CM) ---");
                    telemetry.addData("Forward (Z)", "%.2f cm", forwardDist);
                    telemetry.addData("Side-to-Side (X)", "%.2f cm", sideDist);
                    telemetry.addData("Height (Y)", "%.2f cm", verticalDist);

                    telemetry.addLine("--- RAW ANGLES (DEG) ---");
                    telemetry.addData("Horizontal (TX)", fiducial.getTargetXDegrees());
                    telemetry.addData("Vertical (TY)", fiducial.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Target Status", "No Fiducials in Result");
            }
        } else {
            telemetry.addData("Target Status", "No AprilTag Detected");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}

