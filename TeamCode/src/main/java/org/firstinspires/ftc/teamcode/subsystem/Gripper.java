package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.subsystem.Gripper.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gripper {
    public static class Constants {
        public static Hardware hardware;
        public static Position position;
        private static class Hardware{

            public static Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        }
        private static class Position {
            public static double
                        OPEN = .5,
                        CLOSE = .7;

        }
    }
    private Servo gripper;
    private double servoPosition;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;


    public Gripper (@NonNull OpMode opMode) {
        gamepad2 = opMode.gamepad2;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setDirection(Servo.Direction.FORWARD);

    }
    public void teleOpCommand() {
        if (gamepad2.a) gripper.setPosition(Constants.Position.CLOSE);

        if (gamepad2.b)gripper.setPosition(Constants.Position.OPEN);
    }
}
