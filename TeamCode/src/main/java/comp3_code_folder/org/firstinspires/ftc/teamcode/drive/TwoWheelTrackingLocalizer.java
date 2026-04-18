package comp3_code_folder.org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import comp3_code_folder.org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    // ---- ×§×‘×•×¢×™× ×ž×›×× ×™×™× ×©×œ ×’×œ×’×œ×™ ×”××•×“×•×ž×˜×¨×™×” ----
    public static double TICKS_PER_REV = 2000;   // ×œ×¤×™ ×”×“××˜×”-×©×™×˜ ×©×œ ×”×ž× ×•×¢/×× ×§×•×“×¨ ×©×œ×š
    public static double WHEEL_RADIUS = 0.630;    // ×¨×“×™×•×¡ ×’×œ×’×œ ×”-odo ×‘××™× ×¥'
    public static double GEAR_RATIO = 1;         // ×× ×”×’×œ×’×œ ×ž×—×•×‘×¨ ×™×©×™×¨ ×œ×ž× ×•×¢ â†’ 1

    // ---- ×ž×™×§×•× ×”×’×œ×’×œ×™× ×‘×™×—×¡ ×œ×ž×¨×›×– ×”×¨×•×‘×•×˜ (×¦×™×¨×™× ×©×œ Road Runner) ----
    // PARALLEL_X, Y = ×’×œ×’×œ ×©×ž×•×“×“ ×ª× ×•×¢×” ×§×“×™×ž×”/××—×•×¨×”
    // PERPENDICULAR_X, Y = ×’×œ×’×œ ×©×ž×•×“×“ ×¡×˜×¨×™×™×¤ ×©×ž××œ×”/×™×ž×™× ×”
    public static double PARALLEL_X = 2.086;      // ×ž×¨×—×§ ×‘×¦×™×¨ X ×ž×ž×¨×›×– ×”×¨×•×‘×•×˜
    public static double PARALLEL_Y = -1.220;      // ×ž×¨×—×§ ×‘×¦×™×¨ Y ×ž×ž×¨×›×– ×”×¨×•×‘×•×˜

    public static double PERPENDICULAR_X = -0.748;
    public static double PERPENDICULAR_Y = -1.45669;

    // ×ž×›×¤×™×œ×™ ×§×œ×™×‘×¨×¦×™×” ×œ-X/Y (×ª×§×Ÿ ××™×ª× ××ª ×”×©×’×™××” ×©×œ ××™× ×¦'×™×)
    public static double X_MULTIPLIER = 0.998 ;
    public static double Y_MULTIPLIER = 1.015;
    // ---- ×× ×§×•×“×¨×™× ----
    private Encoder parallelEncoder;       // ×›××Ÿ: rightback
    private Encoder perpendicularEncoder;  // ×›××Ÿ: leftfront

    private final SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                // ×’×œ×’×œ ×©×ž×•×“×“ X (×§×“×™×ž×”/××—×•×¨×”)
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                // ×’×œ×’×œ ×©×ž×•×“×“ Y (×¡×˜×¨×™×™×¤), ×ž×¡×•×‘×‘ ×‘-90Â°
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        // ×©×ž×•×ª ×”×ž× ×•×¢×™× ×‘×“×™×•×§ ×›×ž×• ×‘-Robot Configuration  backleft
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backleft"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backright"));

        // ×× ×’×™×œ×™×ª ×©×›×™×•×•×Ÿ ××—×“ ×”×¤×•×š â€“ ×”×•×¤×›×™× ×›××Ÿ

        parallelEncoder.setDirection(Encoder.Direction.FORWARD);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);


    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    // heading ×ž×”-IMU ×“×¨×š SampleMecanumDrive
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
