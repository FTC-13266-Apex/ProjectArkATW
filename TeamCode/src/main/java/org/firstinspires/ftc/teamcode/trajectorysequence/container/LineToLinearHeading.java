package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class LineToLinearHeading extends PathSegment {
    public double x, y, heading;
    public LineToLinearHeading(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
