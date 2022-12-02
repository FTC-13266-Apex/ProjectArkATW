package org.firstinspires.ftc.teamcode.opmode.sample.roadrunner.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

/**
 * This opmode shows you how to turn 180 degrees left or right
 * @see <a href=https://learnroadrunner.com/advanced.html#_180%C2%B0-turn-direction>Learn Road Runner Advanced 180 Turn Direction</a>
 */
@Autonomous(group = "Z advanced")
public class TurningExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        Drive drive = new Drive(this);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        if (isStopRequested()) return;

        // Turns counter clockwise
        drive.turn(Math.toRadians(180) + 1e-6);

        sleep(2000);

        // Turns clockwise
        drive.turn(Math.toRadians(180) - 1e-6);

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
