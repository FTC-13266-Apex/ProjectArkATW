package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Subsystem {
    private boolean isLocked = false;
    protected OpMode opMode;

    public Subsystem(@NonNull OpMode opmode) {
        this.opMode = opmode;
    }

    protected abstract void manualControl();
    protected void iterative() {};

    /**
     * Checks if the subsystem can be controlled with the gamepad
     * @return True if subsystem is locked, otherwise false
     */
    public boolean isLocked() {
        return isLocked;
    }

    /**
     * Prevents subsystem from being controlled by gamepad
     */
    public void lock() {
        isLocked = true;
    }

    /**
     * Allows Subsystem to be controlled by gamepad
     */
    public void unlock() {
        isLocked = false;
    }

    /**
     * Runs one iteration of the subsystem's manualControl function
     */
    public void runIteratively() {
        if (isLocked()) return;
        manualControl();
        iterative();
    }
}
