package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineTo extends PathSegment {
    public volatile double x, y, endHeading;
    public SplineTo(double x, double y, double endHeading) {
        this.x = x;
        this.y = y;
        this.endHeading = endHeading;
    }
}
