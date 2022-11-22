package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.LiftConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    public static class LiftConstants {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public boolean Left_REVERSED = true;
            public boolean RIGHT_REVERSED = false;
            public double
                    RPM = 1150,
                    CPR = 145.090909;

        }
        public static class Controller {
            public double
                    TOLERANCE     = 8,
                    POSITION_TOLERANCE = 8,
                    KP            = 4,
                    kI            = 0,
                    kD            = 0,
                    kF            = 0;
        }
        public static class Position {
            public int
                    TALL = 955,
                    MIDDLE = 330,
                    LOWER = 0,
                    INITIAL = 0,
                    MAX_POSITION = 1210,
                    MIN_POSITION = 0;
        }
        public static class Speed {
            public double NORMAL        = .5;
            public int MANUAL_MOVE_SPEED = 1;

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
        leftLift = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightMotor");

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);

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
        leftLift.setPower(speed.NORMAL);
        rightLift.setPower(speed.NORMAL);
    }
    public void teleOpCommand() {
        if (gamepad2.dpad_down) moveMotors(position.INITIAL);

        if(gamepad2.dpad_left) moveMotors(position.LOWER);

        if(gamepad2.dpad_right) moveMotors(position.MIDDLE);

        else if (gamepad2.dpad_up) moveMotors(position.TALL);

        if (gamepad2.right_stick_y < -0.3) moveMotors(motorPosition + speed.MANUAL_MOVE_SPEED);

        if (gamepad2.right_stick_y > 0.3) moveMotors(motorPosition - speed.MANUAL_MOVE_SPEED);

        telemetry.addData("slide position", motorPosition);

    }

}
