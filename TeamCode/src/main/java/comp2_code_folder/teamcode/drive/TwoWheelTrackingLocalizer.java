package comp2_code_folder.teamcode.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import comp3_code_folder.org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    // ---- Odometry mechanical constants ----
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.630; // in
    public static double GEAR_RATIO = 1;

    // ---- Wheel positions relative to robot center ----
    public static double PARALLEL_X = 2.086;
    public static double PARALLEL_Y = -1.220;

    public static double PERPENDICULAR_X = -0.748;
    public static double PERPENDICULAR_Y = -1.45669;

    // Calibration multipliers
    public static double X_MULTIPLIER = 0.998;
    public static double Y_MULTIPLIER = 1.015;

    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;

    private final SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backleft"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backright"));

        parallelEncoder.setDirection(Encoder.Direction.FORWARD);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
