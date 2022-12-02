package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class FlipConeWithGamepad extends Command{
    private enum FlipState {
        INITIAL, // lift is at wtv and coneflipper is reset
        LIFT_CONE, // lift is at coneflip position gripper open and coneflipper is down
        FEED_CONE, // cone flipper moves up
        GRAB_CONE,
        GRAB_WAIT,
        RETURN // Move back to initial state
    }

    private final ConeFlipper coneFlipper;
    private final Lift lift;
    private final Gripper gripper;
    private final ElapsedTime waitTimer = new ElapsedTime();

    private FlipState flipState = FlipState.INITIAL;

    public FlipConeWithGamepad(@NonNull LinearOpMode opMode, ConeFlipper coneFlipper, Lift lift, Gripper gripper) {
        super(opMode);
        this.coneFlipper = coneFlipper;
        this.lift = lift;
        this.gripper = gripper;
    }

    @Override
    protected void run() {
        switch (flipState) {
            case INITIAL:
                if (!opMode.gamepad2.y) break; // wait till y is pressed
                gripper.lock();
                lift.lock();
                coneFlipper.lock();

                coneFlipper.drop();
                coneFlipper.getCone();
                gripper.open();
                lift.moveToPickUpFlippedCone();
                flipState = FlipState.LIFT_CONE;
                break;
            case LIFT_CONE:
                if (opMode.gamepad2.y) break; // wait till y is let go
                coneFlipper.lift();
                waitTimer.reset();
                flipState = FlipState.FEED_CONE;
                break;
            case FEED_CONE:
                if (!(waitTimer.seconds() > 1.5)) break; // wait till its been 0.5 second
                coneFlipper.feedCone();
                flipState = FlipState.GRAB_CONE;
                break;
            case GRAB_CONE:
                if (!(waitTimer.seconds() > 2)) break; // wait till its been 0.5 second
                gripper.close();
                waitTimer.reset();
                flipState = FlipState.GRAB_WAIT;
                break;
            case GRAB_WAIT:
                if (!(waitTimer.seconds() > 2)) break; // wait till its been 0.5 second
                lift.moveMid();
                waitTimer.reset();
                flipState = FlipState.RETURN;
                break;
            case RETURN:
                if (!(waitTimer.seconds() > 2)) break; // wait till its been 0.5 second
                coneFlipper.hide();

                gripper.unlock();
                lift.unlock();
                coneFlipper.unlock();

                flipState = FlipState.INITIAL;
                break;
        }
    }
}
