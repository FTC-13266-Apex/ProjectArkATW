package org.firstinspires.ftc.teamcode.trajectorysequence.container;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Vector2dContainer {
    public volatile double x = 0;
    public volatile double y = 0;
    public Vector2dContainer(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d getVector() {
        return new Vector2d(
                x,
                y
        );
    }
}
