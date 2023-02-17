package org.firstinspires.ftc.teamcode.opmode.auto.semiregionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auto.RoadRunnerAuto;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToConstantHeading;
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
            public static double baseAccel = 30; // value
            public static double turnVel = Math.toRadians(100); // value
            public static double turnAccel = Math.toRadians(100); // value
        }

        public static WaitSeconds waitSeconds;
        public static class WaitSeconds {
            public static double dropDriveWait = 0.5;
            public static double dropLiftWait = 0.5;
            public static double pickupLiftWait = 1.0;
        }

        public static PreLoad preLoad;
        public static class PreLoad {
            public static Pose2dContainer startPose = new Pose2dContainer(-38, -66, 90);
            public static LineToLinearHeading a = new LineToLinearHeading(-44, -26, 180);
            static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(a);
        }

        public static Cycle1 cycle1;
        public static class Cycle1 {
            public static SetReversed p1 = new SetReversed(true);
            public static LineToConstantHeading p2 = new LineToConstantHeading(-40, -22);
            public static SplineToConstantHeading p3 = new SplineToConstantHeading(-54, -16, 180);
            public static Forward p4 = new Forward(14);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3, p4);

            public static Back d1 = new Back(1);
            public static LineToSplineHeading d2 = new LineToSplineHeading(-26, -15, -90);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }
        public static Cycle2 cycle2;
        public static class Cycle2 {
            public static SetReversed p1 = new SetReversed(true);
            public static LineToConstantHeading p2 = new LineToConstantHeading(-40, -22);
            public static SplineToConstantHeading p3 = new SplineToConstantHeading(-54, -16, 180);
            public static Forward p4 = new Forward(14);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3, p4);

            public static Back d1 = new Back(1);
            public static LineToSplineHeading d2 = new LineToSplineHeading(-26, -15, -90);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }

        public static Cycle3 cycle3;
        public static class Cycle3 {
            public static SetReversed p1 = new SetReversed(true);
            public static LineToConstantHeading p2 = new LineToConstantHeading(-40, -22);
            public static SplineToConstantHeading p3 = new SplineToConstantHeading(-54, -16, 180);
            public static Forward p4 = new Forward(14);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3, p4);

            public static Back d1 = new Back(1);
            public static LineToSplineHeading d2 = new LineToSplineHeading(-26, -15, -90);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }

        public static Cycle4 cycle4;
        public static class Cycle4 {
            public static SetReversed p1 = new SetReversed(true);
            public static LineToConstantHeading p2 = new LineToConstantHeading(-40, -22);
            public static SplineToConstantHeading p3 = new SplineToConstantHeading(-54, -16, 180);
            public static Forward p4 = new Forward(14);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3, p4);

            public static Back d1 = new Back(1);
            public static LineToSplineHeading d2 = new LineToSplineHeading(-26, -15, -90);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }

        public static Park park;
        public static class Park {
            public static double leftDistance = 36;
            public static double midDistance = 12;
            public static double rightDistance = -12;
            public static volatile double distance = midDistance;

            static TrajectorySequenceContainer getPark() {
                return new TrajectorySequenceContainer(new StrafeRight(distance));
            }
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
            case LEFT: return Constants.Park.leftDistance;
            case MID: return Constants.Park.midDistance;
            case RIGHT: return Constants.Park.rightDistance;
            default: return 0;
        }
    }

    @Override
    public void onStart() {
        Constants.Park.distance = getDistance();
    }

    @Override
    public void run() {
        path.run();
    }
}
