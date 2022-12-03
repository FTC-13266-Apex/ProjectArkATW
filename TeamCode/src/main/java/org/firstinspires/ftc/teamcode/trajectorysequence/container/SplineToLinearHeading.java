package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineToLinearHeading {
    public Pose2dContainer pose2dContainer;
    public double endHeadingDegrees = 0;
    public SplineToLinearHeading(double x, double y, double headingDegrees, double endHeadingDegrees) {
        pose2dContainer = new Pose2dContainer(x, y, headingDegrees);
        this.endHeadingDegrees = endHeadingDegrees;
    }
}
