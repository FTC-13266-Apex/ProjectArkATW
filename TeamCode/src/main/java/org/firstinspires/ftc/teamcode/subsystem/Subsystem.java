package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Subsystem {
    private boolean isLocked;
    protected OpMode opMode;
    public Subsystem(@NonNull OpMode opmode) {
        this.opMode = opmode;
    }

    /**
     * Checks if the subsystem can be controlled with the gamepad
     * @return Whether the subsystem is locked or not
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

    public void controlWithGamepad() {
        if (isLocked()) return;
        manualControl();
    }

    protected abstract void manualControl();
}
