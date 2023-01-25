package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OptimizeStuff extends InstantCommand {
    public OptimizeStuff(@NonNull LinearOpMode opMode) {
        super(opMode);
        opMode.telemetry.setMsTransmissionInterval(50);

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
