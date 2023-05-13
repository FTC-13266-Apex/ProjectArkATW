package org.firstinspires.ftc.teamcode.opmode.auto.semiregionals.left;

import org.firstinspires.ftc.teamcode.opmode.auto.VisionAuto;
import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToConstantHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToSplineHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToConstantHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;

//@Autonomous
public class LeftSemiRegionals extends VisionAuto {
    public static class Constants {
        public static Speed speed;
        public static class Speed {
            public static double baseVel = 25; // value
            public static double baseAccel = 25; // value
            public static double turnVel = Math.toRadians(90); // value
            public static double turnAccel = Math.toRadians(90); // value
            public static double slowVel = baseVel * 0.8; // value
            public static double slowAccel = baseAccel * 0.8; // value
            public static double slowTurnVel = turnVel * 0.8; // value
            public static double slowTurnAccel = turnAccel * 0.8; // value
        }

        public static WaitSeconds waitSeconds;
        public static class WaitSeconds {
            public static double dropDriveWait = 0.5;
            public static double dropLiftWait = 1.3 ;
            public static double pickupLiftWait = 1.0;
            public static double coneFlipperYeetWait = 1.9;
            public static double coneFlipperLiftWait = 0.15;
        }

        public static PreLoad preLoad;
        public static class PreLoad {
            public static Pose2dContainer startPose = new Pose2dContainer(-38, -66, 90);
            public static Forward a = new Forward(7);
            public static LineToSplineHeading b = new LineToSplineHeading(-44, -26, 180);
            static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(a, b);
        }

        public static Cycle1 cycle1;
        public static class Cycle1 {
            public static LineToConstantHeading p1 = new LineToConstantHeading(-40, -22.3);
            public static SplineToConstantHeading p2 = new SplineToConstantHeading(-54.5, -17.5, 180);
            public static Forward p3 = new Forward(13.5);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3);


            public static LineToSplineHeading d1 = new LineToSplineHeading(-27.7, -12.5, -90);
            public static Forward d2 = new Forward(6);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }
        public static Cycle2 cycle2;
        public static class Cycle2 {
            public static Back p1 = new Back(3);
            public static LineToSplineHeading p2 = new LineToSplineHeading(-48, -16, 180);
            public static Forward p3 = new Forward(20.5);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3);

            public static LineToSplineHeading d1 = new LineToSplineHeading(-27.5, -12.5, -90);
            public static Forward d2 = new Forward(7);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }

        public static Cycle3 cycle3;
        public static class Cycle3 {
            public static Back p1 = new Back(2);
            public static LineToSplineHeading p2 = new LineToSplineHeading(-48, Cycle2.d1.y + p1.distance, 180);
            public static Forward p3 = new Forward(20);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3);

            public static Back d1 = new Back(5);
            public static LineToSplineHeading d2 = new LineToSplineHeading(-26, p2.y, -90);
            public static Forward d3 = new Forward(3);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2, d3);
        }

        public static Cycle4 cycle4;
        public static class Cycle4 {
            public static Back p1 = new Back(2);
            public static LineToSplineHeading p2 = new LineToSplineHeading(-48, Cycle3.d2.y + p1.distance, 180);
            public static Forward p3 = new Forward(20);
            static TrajectorySequenceContainer pickup = new TrajectorySequenceContainer(p1, p2, p3);

            public static Back d1 = new Back(3);
            public static LineToSplineHeading d2 = new LineToSplineHeading(-27, -18, -90);
            static TrajectorySequenceContainer drop = new TrajectorySequenceContainer(d1, d2);
        }

        public static Park park;
        public static class Park {
            public static double leftDistance = 36;
            public static double midDistance = 12;
            public static double rightDistance = -12;
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
        ConeFlipper coneFlipper = new ConeFlipper(this);
        path = new Path(
                this,
                drive,
                lift,
                gripper,
                coneFlipper
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
        path.setParkTrajectory(new TrajectorySequenceContainer(new StrafeRight(getDistance())));
    }

    @Override
    public void run() {
        path.run();
    }
}
