package org.firstinspires.ftc.teamcode.opmode.auto.sample.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auto.RoadRunnerAuto;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToLinearHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToSplineHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;

@Autonomous(name = "Experimental Right")
public class Container extends RoadRunnerAuto {
    public static class Constants {
        public static Speed speed;
        public static class Speed {
            public static double baseVel = 35; // value
            public static double baseAccel = 60; // value
            public static double turnVel = 35; // value
            public static double turnAccel = 60; // value
        }

        public static WaitSeconds waitSeconds;
        public static class WaitSeconds {
            public static double dropWait = 1.5;
            public static double pickupLiftWait = 0;
        }

        public static Path path;
        public static class Path {

            public static Pose2dContainer startPose = new Pose2dContainer(31, -62, 90);
            public static double distance = 0;
            public static double leftDistance = 24;
            public static double midDistance = 3;
            public static double rightDistance = -24;


            public static SplineToSplineHeading preload1 = new SplineToSplineHeading(31, -2, 135, 115);
            static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(preload1);

            public static LineToLinearHeading park1 = new LineToLinearHeading(36, -12, 180);
            public static Forward park2 = new Forward(midDistance);
            static TrajectorySequenceContainer park = new TrajectorySequenceContainer(park1, park2);

            public static SplineToSplineHeading cycle1Pickup1 = new SplineToSplineHeading(57, -11, 0, 0);
            public static Forward cycle1Pickup2 = new Forward(7);
            static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(cycle1Pickup1, cycle1Pickup2);

            public static Back cycle1Drop1 = new Back(5);
            public static SplineToSplineHeading cycle1Drop2 = new SplineToSplineHeading(32, -1, 135, 115);
            static TrajectorySequenceContainer cycle1Drop = new TrajectorySequenceContainer(cycle1Drop1, cycle1Drop2);

            public static SplineToSplineHeading cycle2Pickup1 = new SplineToSplineHeading(57, -11, 0, 0);
            public static Forward cycle2Pickup2 = new Forward(7);
            static TrajectorySequenceContainer cycle2Pickup = new TrajectorySequenceContainer(cycle2Pickup1, cycle2Pickup2);

            public static Back cycle2Drop1 = new Back(5);
            public static SplineToSplineHeading cycle2Drop2 = new SplineToSplineHeading(32, -1, 135, 115);
            static TrajectorySequenceContainer cycle2Drop = new TrajectorySequenceContainer(cycle2Drop1, cycle2Drop2);

            public static SplineToSplineHeading cycle3Pickup1 = new SplineToSplineHeading(57, -11, 0, 0);
            public static Forward cycle3Pickup2 = new Forward(7);
            static TrajectorySequenceContainer cycle3Pickup = new TrajectorySequenceContainer(cycle3Pickup1, cycle3Pickup2);

            public static Back cycle3Drop1 = new Back(5);
            public static SplineToSplineHeading cycle3Drop2 = new SplineToSplineHeading(32, -1, 135, 115);
            static TrajectorySequenceContainer cycle3Drop = new TrajectorySequenceContainer(cycle3Drop1, cycle3Drop2);

        }
    }

    private Runner command;

    @Override
    protected boolean usingVision() {
        return true;
    }

    @Override
    public void initialize() {
        Lift lift = new Lift(this);
        Gripper gripper = new Gripper(this);
        command = new Runner(
                this,
                drive,
                lift,
                gripper
        );

        gripper.close();
    }

    public double getDistance() {
        switch (getPosition()) {
            case LEFT: return Constants.Path.leftDistance;
            case MID: return Constants.Path.midDistance;
            case RIGHT: return Constants.Path.rightDistance;
        }
        return 0;
    }

    @Override
    public void onStart() {
        Constants.Path.distance = getDistance();
    }

    @Override
    public void run() {
        command.run();
    }
}
