package org.firstinspires.ftc.teamcode.apollo.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ShooterSubsystem - Firing system including shooter motors and servos.
 * You can adjust velocity and PID parameters to fit your robot! 
 */
public class ShooterSubsystem {
    private DcMotorEx shoot_u, shoot_d;
    private Servo s_block, s_hood;

    public enum ShootState {
        IDLE, OPEN_BLOCK, RUN_TRANSFER
    }

    private ShootState currentShootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();

    // PID and velocity constants - can be adjusted!
    public static double P = 100, I = 1.2, D = 3, F = 0;
    public static double TARGET_RPM = 2250;
    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 130;

    // Servo positions - can be adjusted!
    public static double B_OPEN = 0.556;
    public static double B_CLOSE = 0.501;
    public static double HOOD_OPEN = 0.47;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shoot_u = hardwareMap.get(DcMotorEx.class, "shoot_u");
        shoot_d = hardwareMap.get(DcMotorEx.class, "shoot_d");
        s_block = hardwareMap.get(Servo.class, "s_block");
        s_hood = hardwareMap.get(Servo.class, "s_hood");

        shoot_u.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_d.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot_u.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        updatePIDCoefficients();
    }

    public void updatePIDCoefficients() {
        shoot_u.setVelocityPIDFCoefficients(P, I, D, F);
        shoot_d.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void setShooterPower(boolean on) {
        double targetVel = (TARGET_RPM * TICKS_PER_REV) / 60.0;
        if (on) {
            shoot_u.setVelocity(-targetVel);
            shoot_d.setVelocity(targetVel);
        } else {
            shoot_u.setVelocity(0);
            shoot_d.setVelocity(0);
        }
    }

    public boolean isShooterReady() {
        double rpmU = (Math.abs(shoot_u.getVelocity()) * 60.0) / TICKS_PER_REV;
        double rpmD = (Math.abs(shoot_d.getVelocity()) * 60.0) / TICKS_PER_REV;
        return (Math.abs(TARGET_RPM - rpmU) < RPM_TOLERANCE) &&
               (Math.abs(TARGET_RPM - rpmD) < RPM_TOLERANCE);
    }

    public void update(boolean triggerShoot, IntakeSubsystem intake) {
        s_hood.setPosition(HOOD_OPEN);

        switch (currentShootState) {
            case IDLE:
                s_block.setPosition(B_CLOSE);
                if (triggerShoot && isShooterReady()) {
                    shootTimer.reset();
                    s_block.setPosition(B_OPEN);
                    currentShootState = ShootState.OPEN_BLOCK;
                }
                break;
            case OPEN_BLOCK:
                if (shootTimer.milliseconds() > 300) {
                    if (shootTimer.milliseconds() < 400) {
                        intake.setTransferPower(-0.5);
                    } else {
                        intake.setTransferPower(0.8);
                        shootTimer.reset();
                        currentShootState = ShootState.RUN_TRANSFER;
                    }
                    intake.setIntakePower(-0.8);
                }
                break;
            case RUN_TRANSFER:
                if (shootTimer.milliseconds() > 1000) {
                    intake.stop();
                    s_block.setPosition(B_CLOSE);
                    currentShootState = ShootState.IDLE;
                }
                break;
        }
    }

    public ShootState getCurrentState() {
        return currentShootState;
    }
}
