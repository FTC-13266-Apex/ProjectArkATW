package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class LineToSplineHeading extends PathSegment {
    public double x, y, heading;
    public LineToSplineHeading(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
