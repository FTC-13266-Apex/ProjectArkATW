package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class LineToLinearHeading {
    public Pose2dContainer pose2dContainer;
    public LineToLinearHeading(double x, double y, double heading) {
        pose2dContainer = new Pose2dContainer(x, y, heading);
    }
}
