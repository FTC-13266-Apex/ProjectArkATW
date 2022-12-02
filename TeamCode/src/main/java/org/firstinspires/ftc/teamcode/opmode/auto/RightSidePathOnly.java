package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous
public class RightSidePathOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TrajectoryVelocityConstraint mediumSlowVel = (v, pose2d, pose2d1, pose2d2) -> 40; // value
        TrajectoryAccelerationConstraint mediumSlowAccel = (v, pose2d, pose2d1, pose2d2) -> 50; // value

        TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 30; // value
        TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 30; // value

        Pose2d startPose = new Pose2d(31, -62, Math.toRadians(90));



        Drive drive = new Drive(this);

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(32,-6 , Math.toRadians(135)), Math.toRadians(135))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(59, -12, Math.toRadians(0)), Math.toRadians(0))
                .forward(5)
                .setReversed(false)
                .back(5)
                .splineToSplineHeading(new Pose2d(32,-6 , Math.toRadians(135)), Math.toRadians(135))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(traj1);

        // Put pose in pose storage (so it can be used in teleop)
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
