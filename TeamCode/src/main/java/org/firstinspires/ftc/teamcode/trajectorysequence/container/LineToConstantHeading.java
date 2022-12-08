package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class LineToConstantHeading extends PathSegment {
    public Vector2dContainer vector2dContainer;
    public LineToConstantHeading(double x, double y) {
        vector2dContainer = new Vector2dContainer(x, y);
    }
}
