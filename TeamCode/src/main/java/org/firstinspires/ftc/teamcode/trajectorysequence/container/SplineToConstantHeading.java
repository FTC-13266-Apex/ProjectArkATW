package org.firstinspires.ftc.teamcode.trajectorysequence.container;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class SplineToConstantHeading extends PathSegment {
    public double x, y, endHeading;
    public SplineToConstantHeading(double x, double y, double endHeading) {
        this.x = x;
        this.y = y;
        this.endHeading = endHeading;
    }
}
