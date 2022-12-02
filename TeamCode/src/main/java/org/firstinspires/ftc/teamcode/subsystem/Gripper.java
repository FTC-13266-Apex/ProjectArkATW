package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Gripper extends Subsystem {
    public static class Constants {
        public static Hardware hardware;
        public static Position position;
        public static Sensor sensor;

        private static class Hardware {

            public static Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        }

        private static class Position {
            public static double OPEN = .51;
            public static double CLOSE = .65;
        }

        private static class Sensor {
            public static double GRAB_DISTANCE_MM = 70;
        }
    }

    private final Servo gripper;
    private final ColorRangeSensor distanceSensor;

    public Gripper (@NonNull OpMode opMode) {
        super(opMode);
        gripper = opMode.hardwareMap.get(Servo.class, "gripper");
        gripper.setDirection(Constants.Hardware.DIRECTION);
        distanceSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "distanceSensor");
    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad2.a) close();
        else if (opMode.gamepad2.b) open();
        sensorControl();
    }

    private void sensorControl() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        if ((distance) < Constants.Sensor.GRAB_DISTANCE_MM) close();
        opMode.telemetry.addData("Distance", distance);
    }

    public void open() {
        gripper.setPosition(Constants.Position.OPEN);
    }

    public void close() {
        gripper.setPosition(Constants.Position.CLOSE);
    }
}
