package org.firstinspires.ftc.teamcode.opmode.sample.roadrunner.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously but using
 * inline displacement markers
 * @see  <a href=https://learnroadrunner.com/advanced.html#chaining-async-trajectories>Learn Road Runner Advanced Chaining Async Trajectories</a>
 */
@Autonomous(group = "Z advanced")
public class AsyncChainingAsyncTrajectories extends LinearOpMode {

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(180));

    // Let's define our trajectories
    Trajectory trajectory1;
    Trajectory trajectory2;
    Trajectory trajectory3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize Drive
        Drive drive = new Drive(this);

        // Set initial pose
        drive.setPoseEstimate(startPose);


        // Create a trajectory and have it chained using an inline displacement marker
        trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory2))
                // Basically just tells RR to follow the next trajectory at the end of this one
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineTo(new Vector2d(45, 0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory3))
                .build();


        // Third trajectory
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();

        drive.followTrajectoryAsync(trajectory1);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d poseEstimate = new Pose2d();

        while (opModeIsActive() && !isStopRequested()) {
            // We update drive continuously as it goes through trajectories
            drive.update();

            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        // Write poseEstimate to storage
        PoseStorage.currentPose = poseEstimate;
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    static class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }
}
