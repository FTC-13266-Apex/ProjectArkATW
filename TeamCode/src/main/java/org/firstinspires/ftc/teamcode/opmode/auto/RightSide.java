package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RightSide extends LinearOpMode {
    TrajectoryVelocityConstraint mediumSlowVel = (v, pose2d, pose2d1, pose2d2) -> 30; // value
    TrajectoryAccelerationConstraint mediumSlowAccel = (v, pose2d, pose2d1, pose2d2) -> 30; // value

    TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 40; // value
    TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 40; // value

    Pose2d startPose = new Pose2d(-40, -62, Math.toRadians(90));

    double backDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(this);
        Lift lift = new Lift(this);
        Gripper gripper = new Gripper(this);

        telemetry.setMsTransmissionInterval(50);
        drive.setPoseEstimate(startPose);
        lift.moveInitial();
        gripper.close();

//        vision.init();



//        while (!isStarted() && !isStopRequested())
//        {
//           vision.updateTagOfInterest();
//           vision.printTagData();
//           telemetry.update();
//           if (vision.getTagOfInterest() == null) continue;
//           switch (vision.getTagOfInterest().id) {
//               case 2: {
//                   backDistance = 25;
//                   break;
//               }
//               case 3: {
//                   backDistance = 52;
//                   break;
//               }
//               default: {
//                   backDistance = 3;
//               }
//           }
//        }
        waitForStart();

        TrajectorySequence preLoad = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(35, -30), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(28,-8 , Math.toRadians(120)))
                //drop cone
                //lift down
                .lineToLinearHeading(new Pose2d(43,-12, Math.toRadians(0)))
                //lift up to (new target cone stack pos)
                .splineToLinearHeading(new Pose2d(62, -12, Math.toRadians(0)), Math.toRadians(0))
                //close claw
                .lineToLinearHeading(new Pose2d(43,-12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(28,-8 , Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(43,-12, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(62, -12, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(43,-12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(28,-8 , Math.toRadians(120)))
                //insert park after this
                .lineToLinearHeading(new Pose2d(43,-12, Math.toRadians(0)))
                .build();
        TrajectorySequence boxOne = drive.trajectorySequenceBuilder(preLoad.end())
                .lineToLinearHeading(new Pose2d(-12, -10, Math.toRadians(0)), slowVel, slowAccel)
                .build();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(preLoad);
        //gripper.open();
        drive.followTrajectorySequence(boxOne);
//        lift.initial();
        if (backDistance == 0) return;
        TrajectorySequence forward = drive.trajectorySequenceBuilder(boxOne.end())
                .back(backDistance, slowVel, slowAccel)
                .build();
        drive.followTrajectorySequence(forward);
        sleep(5000);
    }
}
