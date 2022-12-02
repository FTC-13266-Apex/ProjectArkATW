package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Vision extends Subsystem {
    public static class Constants {

    }

    public Vision(@NonNull OpMode opmode) {
        super(opmode);
    }

    @Override
    protected void manualControl() {
        opMode.telemetry.addLine("Um what are you doing? Why is the Vision subsystem being controlled like this?");
    }
}
