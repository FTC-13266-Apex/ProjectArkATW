package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineTo {
    public Vector2dContainer vector2dContainer;
    public double endHeadingDegrees = 0;
    public SplineTo(double x, double y, double endHeadingDegrees) {
        vector2dContainer = new Vector2dContainer(x, y);
        this.endHeadingDegrees = endHeadingDegrees;
    }
}
