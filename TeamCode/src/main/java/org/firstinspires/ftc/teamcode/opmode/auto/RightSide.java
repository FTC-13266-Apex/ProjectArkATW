package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
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
            public static double dropWaitMS = 1500;
            public static Pose2dContainer startPose = new Pose2dContainer(31, -62, 90);
            public static double liftDisplacement = 5;

            public static PreLoad preload;
            public static class PreLoad {
                public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(31, -2, 135, 115);

            }

            public static Park park;
            public static class Park {
                public static LineToLinearHeading lineToLinearHeading = new LineToLinearHeading(36, -12, 180);
                public static double leftDistance = 24;
                public static double midDistance = 3;
                public static double rightDistance = -24;
            }

            public static Cycle1 cycle1;
            public static class Cycle1 {
                public static Pickup pickup;
                public static class Pickup {
                    public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(57, -11, 0, 0);
                    public static double forwardDistance = 7;
                }
                public static Drop drop;
                public static class Drop {
                    public static double backDistance = 5;
                    public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(32, -1, 135, 115);

                }
            }

            public static Cycle2 cycle2;
            public static class Cycle2 {
                public static Pickup pickup;
                public static class Pickup {
                    public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(57, -11, 0, 0);
                    public static double forwardDistance = 7;
                }
                public static Drop drop;
                public static class Drop {
                    public static double backDistance = 5;
                    public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(32, -1, 135, 115);

                }
            }

            public static Cycle3 cycle3;
            public static class Cycle3 {
                public static Pickup pickup;
                public static class Pickup {
                    public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(57, -11, 0, 0);
                    public static double forwardDistance = 7;
                }
                public static Drop drop;
                public static class Drop {
                    public static double backDistance = 5;
                    public static SplineToSplineHeading splineToSplineHeading = new SplineToSplineHeading(32, -1, 135, 115);

                }
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

        Vision vision = new Vision(this);
        telemetry.setMsTransmissionInterval(50);

        gripper.close();

        /* Create start pose based on the Start Pose Constant*/
        Pose2d startPose = new Pose2d(
                Constants.Path.startPose.x,
                Constants.Path.startPose.y,
                Math.toRadians(Constants.Path.startPose.heading)
        );

        /* Trajectory sequences */
        TrajectorySequence preLoad = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineToSplineHeading(Constants.Path.PreLoad.splineToSplineHeading, vel, accel) // The lower the right number is, the more the bot will go right. Higher = more straight of a path
                .build();

        /* Cycle 1 */

        TrajectorySequence cycle1Pickup = drive.trajectorySequenceBuilder(preLoad.end())
                .setReversed(true)
                .addDisplacementMarker(Constants.Path.liftDisplacement, lift::moveCone5)
                .splineToSplineHeading(Constants.Path.Cycle1.Pickup.splineToSplineHeading, vel, accel)
                .forward(Constants.Path.Cycle1.Pickup.forwardDistance, vel, accel)
                .build();

        TrajectorySequence cycle1Drop = drive.trajectorySequenceBuilder(cycle1Pickup.end())
                .setReversed(false)
                .back(Constants.Path.Cycle1.Drop.backDistance, vel, accel)
                .splineToSplineHeading(Constants.Path.Cycle1.Drop.splineToSplineHeading, vel, accel)
                .build();

        /* Cycle 2 */

        TrajectorySequence cycle2Pickup = drive.trajectorySequenceBuilder(preLoad.end())
                .setReversed(true)
                .addDisplacementMarker(Constants.Path.liftDisplacement, lift::moveCone4)
                .splineToSplineHeading(Constants.Path.Cycle2.Pickup.splineToSplineHeading, vel, accel)
                .forward(Constants.Path.Cycle2.Pickup.forwardDistance, vel, accel)
                .build();

        TrajectorySequence cycle2Drop = drive.trajectorySequenceBuilder(cycle1Pickup.end())
                .setReversed(false)
                .back(Constants.Path.Cycle2.Drop.backDistance, vel, accel)
                .splineToSplineHeading(Constants.Path.Cycle2.Drop.splineToSplineHeading, vel, accel)
                .build();

        /* Cycle 3 */

        TrajectorySequence cycle3Pickup = drive.trajectorySequenceBuilder(preLoad.end())
                .setReversed(true)
                .addDisplacementMarker(Constants.Path.liftDisplacement, lift::moveCone3)
                .splineToSplineHeading(Constants.Path.Cycle3.Pickup.splineToSplineHeading, vel, accel)
                .forward(Constants.Path.Cycle3.Pickup.forwardDistance, vel, accel)
                .build();

        TrajectorySequence cycle3Drop = drive.trajectorySequenceBuilder(cycle1Pickup.end())
                .setReversed(false)
                .back(Constants.Path.Cycle3.Drop.backDistance, vel, accel)
                .splineToSplineHeading(Constants.Path.Cycle3.Drop.splineToSplineHeading, vel, accel)
                .build();

        drive.setPoseEstimate(startPose);

        double forwardDistance = 1;
        while (!isStarted() && !isStopRequested()) {
            vision.updateTagOfInterest();
            vision.printTagData();
            telemetry.update();
            if (vision.getTagOfInterest() == null) continue;

            switch (vision.getTagOfInterest().id) {
                case 1: {
                    forwardDistance = Constants.Path.Park.leftDistance;
                    break;
                }
                case 3: {
                    forwardDistance = Constants.Path.Park.rightDistance;
                    break;
                }
                default: {
                    forwardDistance = Constants.Path.Park.midDistance;
                }
            }
        }

        lift.moveHigh();

        // Forward distance cannot equal 0 or trajectory will not generate
        if (forwardDistance == 0) forwardDistance = 1;

        /* Right after we have found out the forward distance, we need to develop the last trajectory */
        /* This ends on cycle3Drop.end */
        TrajectorySequence park = drive.trajectorySequenceBuilder(cycle3Drop.end())
                .lineToLinearHeading(Constants.Path.Park.lineToLinearHeading, vel, accel)
                .forward(forwardDistance)
                .build();

        drive.followTrajectorySequence(preLoad);

        for (int i = 1; i <= 3; i++) { // Code to be looped

            // Drop
            gripper.open();
            sleep((long) Constants.Path.dropWaitMS);

            // Pickup (move lift also)
            switch (i) {
                case 1:
                    drive.followTrajectorySequence(cycle1Pickup);
                    break;
                case 2:
                    drive.followTrajectorySequence(cycle2Pickup);
                    break;
                case 3:
                    drive.followTrajectorySequence(cycle3Pickup);
                    break;
            }

            // After at stack, grab element
            // If the distance sensor detected it, then we know we got here and we can reset pose estimate
            // drive.setPoseEstimate(cycle1Pickup.end());
            gripper.close();
            sleep((long) Constants.Path.dropWaitMS);
            lift.moveHigh();

            switch (i) {
                case 1:
                    drive.followTrajectorySequence(cycle1Drop);
                    break;
                case 2:
                    drive.followTrajectorySequence(cycle2Drop);
                    break;
                case 3:
                    drive.followTrajectorySequence(cycle3Drop);
                    break;
            }
        }
        drive.followTrajectorySequence(park);
        lift.moveInitial();

        // Put pose in pose storage (so it can be used in teleOp)
        // TODO: figure out how to make this work right
        PoseStorage.currentPose = drive.getPoseEstimate();
        sleep(2000);
    }
}