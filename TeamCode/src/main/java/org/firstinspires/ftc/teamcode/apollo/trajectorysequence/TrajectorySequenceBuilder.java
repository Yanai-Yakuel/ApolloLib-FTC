package org.firstinspires.ftc.teamcode.apollo.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder class for creating complex TrajectorySequences.
 */
public class TrajectorySequenceBuilder {
    private final List<Object> sequenceList = new ArrayList<>();
    private final TrajectoryVelocityConstraint baseVelConstraint;
    private final TrajectoryAccelerationConstraint baseAccelConstraint;
    private Pose2d lastPose;
    private double lastTangent;

    public TrajectorySequenceBuilder(Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseMaxAngVel, double baseMaxAngAccel) {
        this.lastPose = startPose;
        this.lastTangent = startPose.getHeading();
        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;
    }

    public TrajectorySequenceBuilder turn(double angle) {
        return this;
    }

    public TrajectorySequenceBuilder addTrajectory(Trajectory trajectory) {
        sequenceList.add(trajectory);
        lastPose = trajectory.end();
        return this;
    }

    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        TrajectoryBuilder builder = new TrajectoryBuilder(lastPose, lastTangent, baseVelConstraint, baseAccelConstraint);
        builder.lineToLinearHeading(endPose);
        Trajectory trajectory = builder.build();
        sequenceList.add(trajectory);
        lastPose = trajectory.end();
        return this;
    }

    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose, TrajectoryVelocityConstraint velConstraint, TrajectoryAccelerationConstraint accelConstraint) {
        TrajectoryBuilder builder = new TrajectoryBuilder(lastPose, lastTangent, velConstraint, accelConstraint);
        builder.lineToLinearHeading(endPose);
        Trajectory trajectory = builder.build();
        sequenceList.add(trajectory);
        lastPose = trajectory.end();
        return this;
    }

    public TrajectorySequence build() {
        return new TrajectorySequence(sequenceList);
    }
}
