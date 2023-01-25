package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineToLinearHeading extends PathSegment {
    public volatile double x, y, heading, endHeading;
    public SplineToLinearHeading(double x, double y, double heading, double endHeading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.endHeading = endHeading;
    }
}
