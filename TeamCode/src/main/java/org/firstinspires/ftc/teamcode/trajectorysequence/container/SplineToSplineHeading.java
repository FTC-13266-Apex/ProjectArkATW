package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineToSplineHeading extends PathSegment {
    public double x, y, heading, endHeading;
    public SplineToSplineHeading(double x, double y, double heading, double endHeading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.endHeading = endHeading;
    }
}
