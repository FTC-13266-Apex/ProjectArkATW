package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Command {
    protected boolean isFinished = false;
    protected LinearOpMode opMode;

    public Command(@NonNull LinearOpMode opMode) {
        this.opMode = opMode;
    }

    protected abstract void run();


    /**
     * Checks if the command has finished yet
     * @return True if command has finished, false otherwise
     */
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * Runs one iteration of the command.
     * Same as {@link #runIteratively()}
     */
    public void runOnce() {
        run();
    }

    /**
     * Runs one iteration of the command.
     * Same as {@link #runOnce()}
     */
    public void runIteratively() {
        run();
    }

    /**
     * Creates a new thread and runs the command iteratively
     * on that thread.
     *
     * @deprecated Use Roadrunner
     */
    @Deprecated
    public void runOnThread() {
        new Thread(() -> {
            while (!isFinished && !opMode.isStopRequested()) {
                runIteratively();
            }
        }).start();
    }
}
