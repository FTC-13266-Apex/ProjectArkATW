package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    public static class Constants {
        public static Hardware hardware;
        public static Controller controller;
        public static Position position;
        public static Speed speed;

        public static class Hardware {
            public static DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
            public static DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
            public static double
                    RPM = 1150,
                    CPR = 145.090909;

        }
        public static class Controller {
            public static double
                    TOLERANCE     = 8,
                    POSITION_TOLERANCE = 8,
                    KP            = 4,
                    kI            = 0,
                    kD            = 0,
                    kF            = 0;
        }
        public static class Position {
            public static int
                    TALL = 955,
                    MIDDLE = 330,
                    LOWER = 0,
                    INITIAL = 0,
                    MAX_POSITION = 1210,
                    MIN_POSITION = 0;
        }
        public static class Speed {
            public static double NORMAL        = .5;
            public static int MANUAL_MOVE_SPEED = 1;

        }
    }
    private DcMotorEx leftLift;
    private  DcMotorEx rightLift;
    private int motorPosition;
    private final  HardwareMap hardwareMap;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;




    public Lift(@NonNull OpMode opMode) {
        gamepad2 = opMode.gamepad2;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        rightLift.setDirection(Hardware.RIGHT_DIRECTION);
        leftLift.setDirection(Hardware.LEFT_DIRECTION);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveMotors(int position){
        this.motorPosition = position;
        leftLift.setTargetPosition(position);
        rightLift.setTargetPosition(position);
        leftLift.setPower(Speed.NORMAL);
        rightLift.setPower(Speed.NORMAL);
    }
    public void teleOpCommand() {
        if (gamepad2.dpad_down) moveMotors(Position.INITIAL);

        if(gamepad2.dpad_left) moveMotors(Position.LOWER);

        if(gamepad2.dpad_right) moveMotors(Position.MIDDLE);

        else if (gamepad2.dpad_up) moveMotors(Position.TALL);

        if (gamepad2.right_stick_y < -0.3) moveMotors(motorPosition + speed.MANUAL_MOVE_SPEED);

        if (gamepad2.right_stick_y > 0.3) moveMotors(motorPosition - speed.MANUAL_MOVE_SPEED);

        telemetry.addData("slide position", motorPosition);

    }

}
