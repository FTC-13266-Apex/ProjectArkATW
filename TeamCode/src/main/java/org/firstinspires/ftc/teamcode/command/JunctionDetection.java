package org.firstinspires.ftc.teamcode.command;

import static org.firstinspires.ftc.teamcode.util.Constants.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class JunctionDetection extends Command {
    private enum SensorState {
        DISABLED,
        JUNCTION,
        RESET
    }

    public static class Constants {
        public static double Grabber_Wait = 0.3;
    }

    private final Gripper gripper;
    private final Lift lift;
    private SensorState sensorState = SensorState.JUNCTION;
    private final ElapsedTime waitTimer = new ElapsedTime();


    public JunctionDetection(@NonNull LinearOpMode opMode, Gripper gripper, Lift lift) {
        super(opMode);
        this.gripper = gripper;
        this.lift = lift;
    }

    @Override
    protected void run() {
        switch (sensorState) {
            case DISABLED:
                if (gripper.noJunction()) break;
                sensorState = SensorState.JUNCTION;
                break;
            case JUNCTION:
                if (!gripper.isInJunction()) break;
                if (!lift.isUp()) break;
                if (!opMode.gamepad1.left_bumper) break;
                gripper.open();
                waitTimer.reset();
                sensorState = SensorState.RESET;
            case RESET:
                if (!(waitTimer.seconds() > Constants.Grabber_Wait)) break;
                lift.moveInitial();
                sensorState = SensorState.DISABLED;
                break;
        }
    }
}
