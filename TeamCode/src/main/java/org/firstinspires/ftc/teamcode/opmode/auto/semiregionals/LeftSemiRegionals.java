package org.firstinspires.ftc.teamcode.opmode.auto.semiregionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auto.RoadRunnerAuto;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToLinearHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToSplineHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SetReversed;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToConstantHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;

@Autonomous
public class LeftSemiRegionals extends RoadRunnerAuto {
    public static class Constants {
        public static Speed speed;
        public static class Speed {
            public static double baseVel = 25; // value
            public static double baseAccel = 60; // value
            public static double turnVel = 60; // value
            public static double turnAccel = 60; // value
        }

        public static WaitSeconds waitSeconds;
        public static class WaitSeconds {
            public static double dropWait = .5;
            public static double pickupLiftWait = 0;
        }

        public static Path path;
        public static class Path {

            public static Pose2dContainer startPose = new Pose2dContainer(-38, -66, 90);
            public static double distance = 0;
            public static double leftDistance = 36;
            public static double midDistance = 12;
            public static double rightDistance = -12;

          //  public static Forward preload1 = new Forward(1);
            public static LineToLinearHeading preload2 = new LineToLinearHeading(-42, -26, 180);
            static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(preload2);

            public static StrafeRight park1 = new StrafeRight(midDistance);
            static TrajectorySequenceContainer park = new TrajectorySequenceContainer(park1);

            public static SetReversed cycle1Pickup1 = new SetReversed(true);
            public static SplineToConstantHeading cycle1Pickup2 = new SplineToConstantHeading(-54, -19, 180);
            public static Forward cycle1Pickup3 = new Forward(10);
            static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(cycle1Pickup1, cycle1Pickup2, cycle1Pickup3);

            public static Back cycle1Drop1 = new Back(1);
            public static LineToSplineHeading cycle1Drop2 = new LineToSplineHeading(-26, -15, -90);
            public static Back cycle1drop3 = new Back(3);
            static TrajectorySequenceContainer cycle1Drop = new TrajectorySequenceContainer(cycle1Drop1, cycle1Drop2, cycle1drop3);

            public static LineToLinearHeading cycle2Pickup1 = new LineToLinearHeading(-64, -19, 180);
//            public static Forward cycle2Pickup2 = new Forward(10);
            static TrajectorySequenceContainer cycle2Pickup = new TrajectorySequenceContainer(cycle2Pickup1);

            public static Back cycle2Drop1 = new Back(1);
            public static LineToSplineHeading cycle2Drop2 = new LineToSplineHeading(-26, -15, -90);
            public static Back cycle2drop3 = new Back(3);
            static TrajectorySequenceContainer cycle2Drop = new TrajectorySequenceContainer(cycle2Drop1, cycle2Drop2, cycle2drop3);

            public static LineToLinearHeading cycle3Pickup1 = new LineToLinearHeading(-64, -19, 180);
//            public static Forward cycle3Pickup2 = new Forward(10);
            static TrajectorySequenceContainer cycle3Pickup = new TrajectorySequenceContainer(cycle3Pickup1);

            public static Back cycle3Drop1 = new Back(1);
            public static LineToSplineHeading cycle3Drop2 = new LineToSplineHeading(-26, -15, -90);
            public static Back cycle3drop3 = new Back(3);
            static TrajectorySequenceContainer cycle3Drop = new TrajectorySequenceContainer(cycle3Drop1, cycle3Drop2, cycle3drop3);

            public static LineToSplineHeading cycle4Pickup1 = new LineToSplineHeading(-56, -19, 180);
//            public static Forward cycle4Pickup2 = new Forward(10);
            static TrajectorySequenceContainer cycle4Pickup = new TrajectorySequenceContainer(cycle4Pickup1);

            public static Back cycle4Drop1 = new Back(1);
            public static LineToSplineHeading cycle4Drop2 = new LineToSplineHeading(-26, -15, -90);
            public static Back cycle4drop3 = new Back(3);
            static TrajectorySequenceContainer cycle4Drop = new TrajectorySequenceContainer(cycle4Drop1, cycle4Drop2, cycle4drop3);

        }
    }

    private Path path;

    @Override
    protected boolean usingVision() {
        return true;
    }

    @Override
    public void initialize() {
        Lift lift = new Lift(this);
        Gripper gripper = new Gripper(this);
        path = new Path(
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
        path.run();
    }
}
