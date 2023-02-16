package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Gripper extends Subsystem {
    public static class Constants {




        public static Hardware hardware;
        private static class Hardware {

            public static Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        }

        public static Position position;
        private static class Position {
            public static volatile double FULL_OPEN = 0.4;
            public static volatile double OPEN = .68;
            public static volatile double CLOSE = .78;
        }

        public static Sensor sensor;
        private static class Sensor {
            public static volatile double GRAB_DISTANCE_MM = 35;
            public static volatile double JUNCTION_DISTANCE_MM = 20;
        }
    }

    private final Servo gripper;
    private final ColorRangeSensor distanceSensor;
    private boolean ignoreSensor = false;
    private boolean noJunction = false;

    public Gripper (@NonNull OpMode opMode) {
        super(opMode);
        gripper = opMode.hardwareMap.get(Servo.class, "gripper");
        gripper.setDirection(Constants.Hardware.DIRECTION);
        distanceSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "distanceSensor");
    }

    @Override
    protected void manualControl() {
        ignoreSensor = true;
        if (opMode.gamepad2.b) open();
        else if (opMode.gamepad2.a) close();
        else ignoreSensor = false;
    }

    protected void disableJunction() {
        if (opMode.gamepad1.dpad_right) {
            noJunction = true;
        } else if (opMode.gamepad1.dpad_left) {
            noJunction = false;
        } else {
            noJunction = false;
        }
    }

    public boolean isInRange() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        opMode.telemetry.addData("Distance", distance);
        return (distance) < Constants.Sensor.GRAB_DISTANCE_MM;
    }

    public boolean isInJunction() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        opMode.telemetry.addData("DistanceJunction", distance);
        return (distance) < Constants.Sensor.JUNCTION_DISTANCE_MM;
    }

    public boolean ignoreSensor() {
        return ignoreSensor;
    }

    public boolean noJunction() {return noJunction;}

    public void fullOpen() {
        gripper.setPosition(Constants.Position.FULL_OPEN);
    }

    public void open() {
        gripper.setPosition(Constants.Position.OPEN);
    }

    public void close() {
        gripper.setPosition(Constants.Position.CLOSE);
    }
}
