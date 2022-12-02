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
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
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
