package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class LineTo extends PathSegment {
    public volatile double x, y;
    public LineTo(double x, double y) {
        this.x = x;
        this.y = y;
    }
}
