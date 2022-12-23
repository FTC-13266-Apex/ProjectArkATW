package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Vision;

public abstract class AutoRunner {
    Drive drive;
    Vision vision;

    public AutoRunner() {

    }

    public abstract void initialize();

    public abstract void run();
}
