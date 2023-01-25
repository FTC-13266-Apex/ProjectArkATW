package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem {
    public static class Constants {
        public static Hardware hardware;
        public static Controller controller;
        public static Position position;
        public static Speed speed;

        public static class Hardware {
            public static DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
            public static DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
            public static final double
                    RPM = 1150,
                    CPR = 145.090909;

        }
        public static class Controller {
            public static volatile double
                    P = 0.008,
                    I = 0,
                    D = 0,
                    F = 0.1;
//            public static int TOLERANCE = 8;
            public static volatile double INTEGRAL_SUM_LIMIT = I == 0 ? 0 : 0.25 / I;
            public static volatile double POWER_LIMIT = 1;
        }
        public static class Position {
            public static volatile int
                    HIGH = 2220,
                    MID = 1611,
                    LOW = 959,
                    GROUND_JUNCTION = 200,
                    INITIAL = 0,
                    FLIPPED_CONE = 470,
                    MAX_POSITION = 3000,
                    MIN_POSITION = -70,
                    AUTO_5CONE = 305,
                    AUTO_4CONE = 223,
                    AUTO_3CONE = 133,
                    AUTO_2CONE = 55;
        }
        public static class Speed {
            public static volatile double NORMAL        = 10;   // ill use this but im testing something
            public static volatile int MANUAL_MOVE_SPEED = 6;

        }
    }
    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;
    private int targetPosition;
    private boolean isMovingManually;

    private final ElapsedTime timeSinceLastRefresh = new ElapsedTime();
    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();




    public Lift(@NonNull OpMode opMode) {

        super(opMode);
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = opMode.hardwareMap.get(DcMotorEx.class, "rightLift");

        rightLift.setDirection(Constants.Hardware.RIGHT_DIRECTION);
        leftLift.setDirection(Constants.Hardware.LEFT_DIRECTION);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
    }

    @Override
    protected void iterative() {
        leftPID();
        rightPID();

        opMode.telemetry.addData("Slide Target Position", targetPosition);
        opMode.telemetry.addData("Slide P", Constants.Controller.P);
        opMode.telemetry.addData("Slide I", Constants.Controller.I);
        opMode.telemetry.addData("Slide D", Constants.Controller.D);
        opMode.telemetry.addData("Slide F", Constants.Controller.F);
        opMode.telemetry.addData("Slide Integral Sum Limit", Constants.Controller.INTEGRAL_SUM_LIMIT);
        timeSinceLastRefresh.reset();
    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad2.dpad_down) moveInitial();
        else if(opMode.gamepad2.dpad_left) moveLow();
        else if(opMode.gamepad2.dpad_right) moveMid();
//        else if (opMode.gamepad2.dpad_up) moveHigh();

        if (opMode.gamepad2.right_stick_y < -0.2 || opMode.gamepad2.right_stick_y > 0.2) {
            changeTargetTo((int)(targetPosition + Constants.Speed.MANUAL_MOVE_SPEED * -opMode.gamepad2.right_stick_y));
            isMovingManually = true;
        } else {
            isMovingManually = false;
        }
    }

    public void updateDashboard() {
        dashboard.addData("Slide Target Position", targetPosition);

        dashboard.addData("Left Slide Measured Position", leftLift.getCurrentPosition());
        dashboard.addData("Left Slide Measured Velocity", leftLift.getVelocity());

        dashboard.addData("Right Slide Measured Position", rightLift.getCurrentPosition());
        dashboard.addData("Right Slide Measured Velocity", rightLift.getVelocity());

        dashboard.addData("Left Slide Power", leftOut);
        dashboard.addData("Right Slide Power", rightOut);
    }

    private double leftLastError, rightLastError;
    private double leftIntegralSum, rightIntegralSum;
    private double leftOut, rightOut;
    public void leftPID() {

        // calculate the error
        double error = targetPosition - leftLift.getCurrentPosition();

        // rate of change of the error
        double derivative = (error - leftLastError) / timeSinceLastRefresh.seconds();

        // sum of all error over time
        leftIntegralSum = leftIntegralSum + (error * timeSinceLastRefresh.seconds());

        double integralSumLimit = Constants.Controller.INTEGRAL_SUM_LIMIT;

        if (integralSumLimit != 0) {
            if (leftIntegralSum > integralSumLimit) {
                leftIntegralSum = integralSumLimit;
            }
            if (leftIntegralSum < -integralSumLimit) {
                leftIntegralSum = -integralSumLimit;
            }
        }

        leftOut =
                (Constants.Controller.P * error) +
                (Constants.Controller.I * leftIntegralSum) +
                (Constants.Controller.D * derivative) +
                (Constants.Controller.F);

        if (leftOut > Constants.Controller.POWER_LIMIT) {
            leftOut = Constants.Controller.POWER_LIMIT;
        }
        if (leftOut < -Constants.Controller.POWER_LIMIT) {
            leftOut = -Constants.Controller.POWER_LIMIT;
        }
        leftLift.setPower(leftOut);

        leftLastError = error;
    }

    public void rightPID() {

        // calculate the error
        double error = targetPosition - rightLift.getCurrentPosition();

        // rate of change of the error
        double derivative = (error - rightLastError) / timeSinceLastRefresh.seconds();

        // sum of all error over time
        rightIntegralSum = rightIntegralSum + (error * timeSinceLastRefresh.seconds());

        double integralSumLimit = Constants.Controller.INTEGRAL_SUM_LIMIT;

        if (integralSumLimit != 0) {
            if (leftIntegralSum > integralSumLimit) {
                leftIntegralSum = integralSumLimit;
            }
            if (leftIntegralSum < -integralSumLimit) {
                leftIntegralSum = -integralSumLimit;
            }
        }

        rightOut =
                (Constants.Controller.P * error) +
                (Constants.Controller.I * rightIntegralSum) +
                (Constants.Controller.D * derivative) +
                (Constants.Controller.F);

        if (rightOut > Constants.Controller.POWER_LIMIT) {
            rightOut = Constants.Controller.POWER_LIMIT;
        }
        if (rightOut < -Constants.Controller.POWER_LIMIT) {
            rightOut = -Constants.Controller.POWER_LIMIT;
        }
        rightLift.setPower(rightOut);

        rightLastError = error;
    }

    public void changeTargetTo(int position) {
        if (position > Constants.Position.MAX_POSITION || position < Constants.Position.MIN_POSITION) return;
        this.targetPosition = position;
    }

    public void moveInitial() {
        if (isMovingManually) {
            Constants.Position.INITIAL = targetPosition;
        }
        changeTargetTo(Constants.Position.INITIAL);
    }

    public void moveHigh() {
        if (isMovingManually) {
            Constants.Position.HIGH = targetPosition;
        }
        changeTargetTo(Constants.Position.HIGH);
    }

    public void moveMid() {
        if (isMovingManually) {
            Constants.Position.MID = targetPosition;
        }
        changeTargetTo(Constants.Position.MID);
    }

    public void moveLow() {
        if (isMovingManually) {
            Constants.Position.LOW = targetPosition;
        }
        changeTargetTo(Constants.Position.LOW);
    }

    public void moveGroundJunction() {
        changeTargetTo(Constants.Position.GROUND_JUNCTION);
    }

    public void moveCone5() {
        changeTargetTo(Constants.Position.AUTO_5CONE);
    }

    public void moveCone4() {
        changeTargetTo(Constants.Position.AUTO_4CONE);
    }

    public void moveCone3() {
        changeTargetTo(Constants.Position.AUTO_3CONE);
    }

    public void moveCone2() {
        changeTargetTo(Constants.Position.AUTO_2CONE);
    }

    public void moveToPickUpFlippedCone() {
        changeTargetTo(Constants.Position.FLIPPED_CONE);
    }

    public boolean isDown() {
        return targetPosition <= Constants.Position.INITIAL;
    }
}
