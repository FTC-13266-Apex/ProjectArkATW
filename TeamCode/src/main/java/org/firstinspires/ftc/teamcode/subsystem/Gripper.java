package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends Subsystem {
    public static class Constants {
        public static Hardware hardware;
        public static Position position;

        private static class Hardware {

            public static Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        }

        private static class Position {
            public static double
                    OPEN = .51,
                    CLOSE = .65;


        }
    }

    private final Servo gripper;

    public Gripper (@NonNull OpMode opMode) {
        super(opMode);
        gripper = opMode.hardwareMap.get(Servo.class, "gripper");
        gripper.setDirection(Constants.Hardware.DIRECTION);
    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad2.a) close();
        else if (opMode.gamepad2.b) open();
    }

    public void open() {
        gripper.setPosition(Constants.Position.OPEN);
    }

    public void close() {
        gripper.setPosition(Constants.Position.CLOSE);
    }
}
