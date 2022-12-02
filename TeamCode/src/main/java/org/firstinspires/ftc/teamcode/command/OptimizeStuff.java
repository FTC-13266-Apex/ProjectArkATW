package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OptimizeStuff extends Command {
    public OptimizeStuff(@NonNull LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    protected void run() {
        opMode.telemetry.setMsTransmissionInterval(50);
    }
}
