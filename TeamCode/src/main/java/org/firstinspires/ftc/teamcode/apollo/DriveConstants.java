package org.firstinspires.ftc.teamcode.apollo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * DriveConstants - Configuration for robot driving parameters.
 */
@Config
public class DriveConstants {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = true;

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;

    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static double WHEEL_RADIUS = 1.96850; // in
    public static double GEAR_RATIO = 1.125;
    public static double TRACK_WIDTH = 12.28346;

    public static double MAX_VEL = 85.74561;
    public static double MAX_ACCEL = 85.74561;
    public static double MAX_ANG_VEL = Math.toRadians(399.9693);
    public static double MAX_ANG_ACCEL = Math.toRadians(399.9693);

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
