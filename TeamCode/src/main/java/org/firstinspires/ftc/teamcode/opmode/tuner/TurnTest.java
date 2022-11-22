package org.firstinspires.ftc.teamcode.opmode.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

import static org.firstinspires.ftc.teamcode.opmode.tuner.TurnTest.Constants.*;
/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static class Constants {
        public static double ANGLE = 90; // deg
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(this);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
