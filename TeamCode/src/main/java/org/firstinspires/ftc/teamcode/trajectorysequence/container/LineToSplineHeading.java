package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class LineToSplineHeading extends PathSegment {
    public Pose2dContainer pose2dContainer;
    public LineToSplineHeading(double x, double y, double heading) {
        pose2dContainer = new Pose2dContainer(x, y, heading);
    }
}
