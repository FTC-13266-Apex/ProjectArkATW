package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToLinearHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToSplineHeading;

@Autonomous
public class RightSide extends LinearOpMode {
    public static class Constants {
        public static Speed speed;
        public static class Speed {
            public static double velocity = 35; // value
            public static double acceleration = 60; // value
        }

        public static Path path;
        public static class Path {
            public static Pose2dContainer startPose = new Pose2dContainer(31, -62, 90);
            public static PreLoad preload;
            public static class PreLoad {
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(31, -1, 135, 115);

            }
            public static Cycle1Pickup cycle1Pickup;
            public static class Cycle1Pickup {
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(57, -11, 0, 0);
                public static double forwardDistance = 7;
            }
            public static Cycle1Drop cycle1Drop;
            public static class Cycle1Drop {
                public static double backDistance = 5;
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(32, -1, 135, 115);

            }
            public static Park park;
            public static class Park {
                public static LineToLinearHeading lineToLinearHeading = new LineToLinearHeading(36, -12, 180);
                public static double parkDistance = 24;
            }
            public static double dropWaitMS = 1500;
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
                .addDisplacementMarker(5, lift::moveCone5)
                .splineToSplineHeading(Constants.Path.Cycle1Pickup.splineToSplineHeading, vel, accel)
                .forward(Constants.Path.Cycle1Pickup.forwardDistance, vel, accel)
                .build();

        TrajectorySequence cycle1Drop = drive.trajectorySequenceBuilder(cycle1Pickup.end())
                .setReversed(false)
                .back(Constants.Path.Cycle1Drop.backDistance, vel, accel)
                .splineToSplineHeading(Constants.Path.Cycle1Drop.splineToSplineHeading, vel, accel)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(cycle1Drop.end())
                .lineToLinearHeading(Constants.Path.Park.lineToLinearHeading, vel, accel)
                .build();

        TrajectorySequence parkMove = null;
        if (Constants.Path.Park.parkDistance != 0) {
            parkMove = drive.trajectorySequenceBuilder(park.end())
                    .forward(Constants.Path.Park.parkDistance)
                    .build();
        }



        drive.setPoseEstimate(startPose);


        waitForStart();
        if (isStopRequested()) return;

        lift.moveHigh();
        drive.followTrajectorySequence(preLoad);

        for (int i = 1; i <= 2; i++) { // Code to be looped
            gripper.open();
            sleep((long) Constants.Path.dropWaitMS);
//            switch (i) {
//                case 1:
//                    lift.moveCone5();
//                    break;
//                case 2:
//                    lift.moveCone4();
//                    break;
//                case 3:
//                    lift.moveCone3();
//                   break;
//                case 4:
//                    lift.moveCone2();
//                    break;
//            }


            drive.followTrajectorySequence(cycle1Pickup);

            gripper.close();
            // If the distance sensor detected it, then we know we got here and we can reset pose estimate
            // drive.setPoseEstimate(cycle1Pickup.end());
            sleep((long) Constants.Path.dropWaitMS);
            lift.moveHigh();

            drive.followTrajectorySequence(cycle1Drop);
        }
        drive.followTrajectorySequence(park);

        if (parkMove != null) {
            drive.followTrajectorySequence(parkMove);
        }

        lift.moveInitial();
        sleep(2000);

        // Put pose in pose storage (so it can be used in teleOp)
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
