package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineToLinearHeading {
    public double x = 0;
    public double y = 0;
    public double heading = 0;
    public double endHeading = 0;
    public SplineToLinearHeading(double x, double y, double heading, double endHeading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.endHeading = endHeading;
    }
}
