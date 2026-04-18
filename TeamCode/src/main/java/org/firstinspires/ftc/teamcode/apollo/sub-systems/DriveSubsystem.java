package org.firstinspires.ftc.teamcode.apollo.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.apollo.SampleMecanumDrive;

/**
 * DriveSubsystem - Drive helper with Limelight pose correction.
 */
public class DriveSubsystem {
    public final SampleMecanumDrive drive;
    private Limelight3A limelight;

    public static double HOLD_KP = 0.045;
    public static double HOLD_KI = 0.003;
    public static double HOLD_KD = 0.18;
    public static double HOLD_KH = 1.2;

    private double xIntegral = 0;
    private double yIntegral = 0;
    public static final double M_TO_IN = 39.37;

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(0);
                limelight.start();
            }
        } catch (Exception e) {
            limelight = null;
        }
    }

    public void update() {
        drive.update();

        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        if (result != null && result.isValid()) {
            updateBotPose(result);
        }
    }

    private void updateBotPose(LLResult result) {
        Pose3D botPose = result.getBotpose();
        if (botPose != null && botPose.getPosition() != null) {
            double x = botPose.getPosition().x * M_TO_IN;
            double y = botPose.getPosition().y * M_TO_IN;
            double yaw = Math.toRadians(botPose.getOrientation().getYaw());
            drive.setPoseEstimate(new Pose2d(x, y, yaw));
        }
    }

    public void alignToTarget(Pose2d target) {
        Pose2d currentPose = drive.getPoseEstimate();
        Pose2d velocity = drive.getPoseVelocity();
        if (velocity == null) {
            velocity = new Pose2d();
        }

        double xErr = target.getX() - currentPose.getX();
        double yErr = target.getY() - currentPose.getY();
        double hErr = Angle.normDelta(target.getHeading() - currentPose.getHeading());
        double distance = Math.hypot(xErr, yErr);

        if (distance < 0.1 && Math.abs(hErr) < Math.toRadians(1)) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            xIntegral = 0;
            yIntegral = 0;
            return;
        }

        if (Math.abs(xErr) < 5) {
            xIntegral = Math.max(-10, Math.min(10, xIntegral + xErr * 0.02));
        } else {
            xIntegral = 0;
        }

        if (Math.abs(yErr) < 5) {
            yIntegral = Math.max(-10, Math.min(10, yIntegral + yErr * 0.02));
        } else {
            yIntegral = 0;
        }

        double xCmd = xErr * HOLD_KP + xIntegral * HOLD_KI - velocity.getX() * (HOLD_KD * 0.5);
        double yCmd = yErr * HOLD_KP + yIntegral * HOLD_KI - velocity.getY() * (HOLD_KD * 0.5);
        double maxSpeed = Math.min(0.8, distance * 0.15 + 0.2);

        double heading = currentPose.getHeading();
        double rotX = xCmd * Math.cos(-heading) - yCmd * Math.sin(-heading);
        double rotY = xCmd * Math.sin(-heading) + yCmd * Math.cos(-heading);

        rotX = Math.max(-maxSpeed, Math.min(maxSpeed, rotX));
        rotY = Math.max(-maxSpeed, Math.min(maxSpeed, rotY));

        double rotPower = Math.max(-0.5, Math.min(0.5, hErr * HOLD_KH));
        drive.setWeightedDrivePower(new Pose2d(rotX, rotY, rotPower));
    }

    public boolean isTargetVisible() {
        if (limelight == null) {
            return false;
        }
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public Pose2d getPoseVelocity() {
        Pose2d velocity = drive.getPoseVelocity();
        return velocity != null ? velocity : new Pose2d();
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }

    public double getRawExternalHeading() {
        return drive.getRawExternalHeading();
    }

    public void stopLimelight() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
