package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class FlipCone {
    private enum FlipState {
        INITIAL, // lift is at wtv and coneflipper is reset
        GET_CONE, // lift is at coneflip position gripper open and coneflipper is down
        FEED_CONE, // cone flipper moves up
        RETURN // Move back to initial state

    }

    private final ConeFlipper coneFlipper;
    private final Lift lift;
    private final Gripper gripper;
    private final ElapsedTime waitTimer = new ElapsedTime();

    private FlipState flipState = FlipState.INITIAL;

    public FlipCone(ConeFlipper coneFlipper, Lift lift, Gripper gripper) {
        this.coneFlipper = coneFlipper;
        this.lift = lift;
        this.gripper = gripper;
    }

    public void runCommandManual(Gamepad gamepad2) {
        switch (flipState) {
            case INITIAL:
                if (!gamepad2.y) break; // wait till y is pressed
                gripper.lock();
                lift.lock();
                coneFlipper.lock();

                coneFlipper.drop();
                coneFlipper.getCone();
                gripper.open();
                lift.moveToPickUpFlippedCone();
                flipState = FlipState.GET_CONE;
                break;
            case GET_CONE:
                if (gamepad2.y) break; // wait till y is let go
                coneFlipper.lift();
                coneFlipper.feedCone();
                waitTimer.reset();
                flipState = FlipState.FEED_CONE;
                break;
            case FEED_CONE:
                if (!(waitTimer.seconds() > 0.5)) break; // wait till its been 0.5 second
                gripper.close();
                lift.moveMid();
                waitTimer.reset();
                flipState = FlipState.RETURN;
                break;
            case RETURN:
                if (!(waitTimer.seconds() > 0.5)) break; // wait till its been 0.5 second
                coneFlipper.hide();

                gripper.unlock();
                lift.unlock();
                coneFlipper.unlock();

                flipState = FlipState.INITIAL;
                break;
        }
    }
}
