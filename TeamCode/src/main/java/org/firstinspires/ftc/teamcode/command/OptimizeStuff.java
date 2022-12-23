package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OptimizeStuff extends InstantCommand {
    public OptimizeStuff(@NonNull LinearOpMode opMode) {
        super(opMode);
        opMode.telemetry.setMsTransmissionInterval(50);
    }
}
