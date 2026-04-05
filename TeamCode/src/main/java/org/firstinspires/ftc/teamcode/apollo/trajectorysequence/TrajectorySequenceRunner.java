package org.firstinspires.ftc.teamcode.apollo.trajectorysequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

/**
 * Runner class that handles execution of TrajectorySequences.
 */
@Config
public class TrajectorySequenceRunner {
    private TrajectoryFollower follower;

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPid, VoltageSensor batteryVoltageSensor, List<Integer> lastEncPositions, List<Integer> lastEncVels, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        this.follower = follower;
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        for (int i = 0; i < trajectorySequence.size(); i++) {
            Object obj = trajectorySequence.get(i);
            if (obj instanceof Trajectory) {
                follower.followTrajectory((Trajectory) obj);
            }
        }
    }

    public DriveSignal update(Pose2d pose, Pose2d poseVelocity) {
        follower.update(pose);
        return new DriveSignal(new Pose2d(), new Pose2d());
    }

    public Pose2d getLastPoseError() {
        return new Pose2d();
    }

    public boolean isBusy() {
        return false;
    }
}
