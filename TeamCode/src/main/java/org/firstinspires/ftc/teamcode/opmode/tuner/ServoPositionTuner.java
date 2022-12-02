package org.firstinspires.ftc.teamcode.opmode.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoPositionTuner extends LinearOpMode {
    public static class Constants {
        public static String name = "coneFlipperBottom";
        public static double positionChangeSpeed = 0.0001;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, Constants.name);
        double position = 0;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                position += Constants.positionChangeSpeed;
                servo.setPosition(position);
            }
            else if (gamepad1.b) {
                position -= Constants.positionChangeSpeed;
                servo.setPosition(position);
            }
            telemetry.addData("Tuning Servo", Constants.name);
            telemetry.addData("Position", position);
        }

    }
}

