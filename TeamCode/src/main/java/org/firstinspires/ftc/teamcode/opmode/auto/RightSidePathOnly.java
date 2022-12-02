package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToSplineHeading;

@Autonomous
public class RightSidePathOnly extends LinearOpMode {
    public static class Constants {
        public static Speed speed;
        public static class Speed {
            public static double velocity = 50; // value
            public static double acceleration = 60; // value
        }

        public static Path path;
        public static class Path {
            public static Pose2dContainer startPose = new Pose2dContainer(31, -62, 90);
            public static PreLoad preload;
            public static class PreLoad {
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(32, -6, 135, 115);
            }
            public static Cycle1Pickup cycle1Pickup;
            public static class Cycle1Pickup {
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(59, -12, 0, 135);
                public static double forwardDistance = 5;
            }
            public static Cycle1Drop cycle1Drop;
            public static class Cycle1Drop {
                public static double backDistance = 5;
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(32, -6, 135, 135);

            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TrajectoryVelocityConstraint vel = (v, pose2d, pose2d1, pose2d2) -> Constants.Speed.velocity; // value
        TrajectoryAccelerationConstraint accel = (v, pose2d, pose2d1, pose2d2) -> Constants.Speed.acceleration; // value


        Drive drive = new Drive(this);
        Lift lift = new Lift(this);
        Gripper gripper = new Gripper(this);

        gripper.close();

        Pose2d startPose = new Pose2d(
                Constants.Path.startPose.x,
                Constants.Path.startPose.y,
                Math.toRadians(Constants.Path.startPose.heading));

        TrajectorySequence preLoad = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineToSplineHeading(Constants.Path.PreLoad.splineToSplineHeading, vel, accel) // The lower the right number is, the more the bot will go right. Higher = more straight of a path
                .build();

        TrajectorySequence cycle1Pickup = drive.trajectorySequenceBuilder(preLoad.end())
                .setReversed(true)
                .splineToSplineHeading(Constants.Path.Cycle1Pickup.splineToSplineHeading, vel, accel)
                .forward(Constants.Path.Cycle1Pickup.forwardDistance, vel, accel)
                .build();

        TrajectorySequence cycle1Drop = drive.trajectorySequenceBuilder(cycle1Pickup.end())
                .setReversed(false)
                .back(Constants.Path.Cycle1Drop.backDistance, vel, accel)
                .splineToSplineHeading(Constants.Path.Cycle1Drop.splineToSplineHeading, vel, accel)
                .build();

        drive.setPoseEstimate(startPose);


        waitForStart();
        if (isStopRequested()) return;

        lift.moveHigh();
        drive.followTrajectorySequence(preLoad);

        for (int i = 0; i < 1; i++) { // Code to be looped
            gripper.open();
            sleep(2000);
            lift.moveCone5();

            drive.followTrajectorySequence(cycle1Pickup);

            gripper.close();
            // If the distance sensor detected it, then we know we got here and we can reset pose estimate
            // drive.setPoseEstimate(cycle1Pickup.end());
            sleep(2000);
            lift.moveHigh();

            drive.followTrajectorySequence(cycle1Drop);
        }

        // Put pose in pose storage (so it can be used in teleOp)
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
