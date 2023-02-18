package org.firstinspires.ftc.teamcode.opmode.auto.semiregionals.right;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;

public class Path extends Command {

    enum AutoState {
        PRELOAD,
        CONE_YEET,
        CONE_FLIPPER_LIFT,
        PRELOAD_DROP,
        CYCLE_START,
        CYCLE,
        PARK,
        PARK_LIFT_WAIT,
        END
    }

    enum CycleState {
        PICKUP_PATH,
        PICKUP_LIFT,
        PICKUP,
        DROP_PATH,
        DROP
    }

    private final TrajectorySequence preLoad;
    private final TrajectorySequence cycle1Pickup;
    private final TrajectorySequence cycle1Drop;
    private final TrajectorySequence cycle2Pickup;
    private final TrajectorySequence cycle2Drop;
    private final TrajectorySequence cycle3Pickup;
    private final TrajectorySequence cycle3Drop;
    private final TrajectorySequence cycle4Pickup;
    private final TrajectorySequence cycle4Drop;
    private final Drive drive;
    private final Lift lift;
    private final Gripper gripper;
    private final ConeFlipper coneFlipper;
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final TrajectorySequenceConstraints constraints = new TrajectorySequenceConstraints(
            RightSemiRegionals.Constants.Speed.baseVel,
            RightSemiRegionals.Constants.Speed.baseAccel,
            RightSemiRegionals.Constants.Speed.turnVel,
            RightSemiRegionals.Constants.Speed.turnAccel
    );
    private final TrajectorySequenceConstraints slowConstraints = new TrajectorySequenceConstraints(
            RightSemiRegionals.Constants.Speed.slowVel,
            RightSemiRegionals.Constants.Speed.slowAccel,
            RightSemiRegionals.Constants.Speed.slowTurnVel,
            RightSemiRegionals.Constants.Speed.slowTurnAccel
    );

    private TrajectorySequence currentPickupTrajectorySequence;
    private TrajectorySequence currentDropTrajectorySequence;
    private Runnable currentLiftCommand;
    private int cycleNumber = 1;
    private TrajectorySequenceContainer parkTrajectory = new TrajectorySequenceContainer(new StrafeRight(RightSemiRegionals.Constants.Park.rightDistance));

    private AutoState autoState = AutoState.PRELOAD;
    private CycleState cycleState = CycleState.PICKUP_PATH;

    public Path(@NonNull LinearOpMode opMode, Drive drive, Lift lift, Gripper gripper, ConeFlipper coneFlipper) {
        super(opMode);
        this.drive = drive;
        this.lift = lift;
        this.gripper = gripper;
        this.coneFlipper = coneFlipper;

        drive.setPoseEstimate(RightSemiRegionals.Constants.PreLoad.startPose.getPose());


        preLoad = RightSemiRegionals.Constants.PreLoad.preload.build(RightSemiRegionals.Constants.PreLoad.startPose.getPose(), constraints);

        cycle1Pickup = RightSemiRegionals.Constants.Cycle1.pickup.build(preLoad.end(), slowConstraints);
        cycle1Drop = RightSemiRegionals.Constants.Cycle1.drop.build(cycle1Pickup.end(), constraints);

        cycle2Pickup = RightSemiRegionals.Constants.Cycle2.pickup.build(cycle1Drop.end(), slowConstraints);
        cycle2Drop = RightSemiRegionals.Constants.Cycle2.drop.build(cycle2Pickup.end(), constraints);

        cycle3Pickup = RightSemiRegionals.Constants.Cycle3.pickup.build(cycle2Drop.end(), constraints);
        cycle3Drop = RightSemiRegionals.Constants.Cycle3.drop.build(cycle3Pickup.end(), constraints);

        cycle4Pickup = RightSemiRegionals.Constants.Cycle4.pickup.build(cycle3Drop.end(), constraints);
        cycle4Drop = RightSemiRegionals.Constants.Cycle4.drop.build(cycle4Pickup.end(), constraints);
    }

    @Override
    protected void run() {
        switch (autoState) {
            case PRELOAD:
                if (drive.isBusy()) break;
                drive.followTrajectorySequenceAsync(preLoad);
                lift.moveLow();
                coneFlipper.drop();
                coneFlipper.SignalConeYeet();

                waitTimer.reset();
                autoState = AutoState.CONE_YEET;
                break;
            case CONE_YEET:
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.coneFlipperYeetWait) break;
                coneFlipper.SignalConePusher();
                waitTimer.reset();
                autoState = AutoState.CONE_FLIPPER_LIFT;
                break;
            case CONE_FLIPPER_LIFT:
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.coneFlipperLiftWait) break;
                coneFlipper.lift();
                coneFlipper.hide();
                autoState = AutoState.PRELOAD_DROP;
                break;
            case PRELOAD_DROP:
                if (drive.isBusy()) break;
                gripper.open();

                waitTimer.reset();
                autoState = AutoState.CYCLE_START;
                break;

            case CYCLE_START:
                switch (cycleNumber) {
                    case 1:
                        currentPickupTrajectorySequence = cycle1Pickup;
                        currentDropTrajectorySequence = cycle1Drop;
                        currentLiftCommand = lift::moveCone5;
                        autoState = AutoState.CYCLE;
                        break;
                    case 2:
                        currentPickupTrajectorySequence = cycle2Pickup;
                        currentDropTrajectorySequence = cycle2Drop;
                        currentLiftCommand = lift::moveCone4;
                        autoState = AutoState.CYCLE;
                        break;
//                    case 3:
//                        currentPickupTrajectorySequence = cycle3Pickup;
//                        currentDropTrajectorySequence = cycle3Drop;
//                        currentLiftCommand = lift::moveCone3;
//                        autoState = AutoState.CYCLE;
//                        break;
//                    case 4:
//                        currentPickupTrajectorySequence = cycle4Pickup;
//                        currentDropTrajectorySequence = cycle4Drop;
//                        currentLiftCommand = lift::moveCone2;
//                        autoState = AutoState.CYCLE;
//                        break;
                    default:
                        autoState = AutoState.PARK;
                        break;
                }
                break;

            case CYCLE:
                cycle();
                break;

            case PARK:
                if (drive.isBusy()) break;
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.dropDriveWait) break;
                drive.followTrajectorySequenceAsync(
                        //TODO make srue to update this if you add more cycles!!!!!!
                        parkTrajectory.build(cycle1Drop.end(), constraints));
                autoState = AutoState.PARK_LIFT_WAIT;
                waitTimer.reset();
                break;
            case PARK_LIFT_WAIT:
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.dropLiftWait) break;
                lift.moveInitial();
                waitTimer.reset();
                break;
            case END:
                if (drive.isBusy()) break;
                if (waitTimer.seconds() < 2) break;
                opMode.stop();
                break;
        }

        drive.update();
        // lift.update();
    }

    public void cycle() {
        switch (cycleState) {
            case PICKUP_PATH:
                if (drive.isBusy()) break;
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.dropDriveWait) break;
                drive.followTrajectorySequenceAsync(currentPickupTrajectorySequence);
                waitTimer.reset();
                cycleState = CycleState.PICKUP_LIFT;
                break;
            case PICKUP_LIFT:
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.dropLiftWait) break;
                currentLiftCommand.run();

                cycleState = CycleState.PICKUP;
                break;
            case PICKUP:
                if (drive.isBusy()) break;
                gripper.close();

                waitTimer.reset();
                cycleState = CycleState.DROP_PATH;
                break;
            case DROP_PATH:
                if (drive.isBusy()) break;
                if (waitTimer.seconds() < RightSemiRegionals.Constants.WaitSeconds.pickupLiftWait) break;
                lift.moveMid();
                drive.followTrajectorySequenceAsync(currentDropTrajectorySequence);

                cycleState = CycleState.DROP;
                break;
            case DROP:
                if (drive.isBusy()) break;
                gripper.open();

                waitTimer.reset();

                cycleNumber++;
                autoState = AutoState.CYCLE_START;
                cycleState = CycleState.PICKUP_PATH;
                break;
        }
    }

    public void setParkTrajectory(TrajectorySequenceContainer parkTrajectory) {
        this.parkTrajectory = parkTrajectory;
    }
}
