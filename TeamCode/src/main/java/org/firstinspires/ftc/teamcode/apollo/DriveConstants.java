package org.firstinspires.ftc.teamcode.apollo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = true;
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.96850;
    public static double GEAR_RATIO = 1.125;
    public static double TRACK_WIDTH = 12.28346;

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 85.74561;
    public static double MAX_ACCEL = 85.74561;
    public static double MAX_ANG_VEL = Math.toRadians(399.9693);
    public static double MAX_ANG_ACCEL = Math.toRadians(399.9693);

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
