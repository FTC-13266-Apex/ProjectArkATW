package org.firstinspires.ftc.teamcode.opmode.auto.semiregionals;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;

public class Path extends Command {

    enum AutoState {
        PRELOAD,
        PRELOAD_DROP,
        CYCLE_START,
        CYCLE,
        PARK,
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
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final TrajectorySequenceConstraints constraints = new TrajectorySequenceConstraints(
            LeftSemiRegionals.Constants.Speed.baseVel,
            LeftSemiRegionals.Constants.Speed.baseAccel,
            LeftSemiRegionals.Constants.Speed.turnVel,
            LeftSemiRegionals.Constants.Speed.turnAccel
    );

    private TrajectorySequence currentPickupTrajectorySequence;
    private TrajectorySequence currentDropTrajectorySequence;
    private Runnable currentLiftCommand;
    private int cycleNumber = 1;

    private AutoState autoState = AutoState.PRELOAD;
    private CycleState cycleState = CycleState.PICKUP_PATH;

    public Path(@NonNull LinearOpMode opMode, Drive drive, Lift lift, Gripper gripper) {
        super(opMode);
        this.drive = drive;
        this.lift = lift;
        this.gripper = gripper;

        drive.setPoseEstimate(LeftSemiRegionals.Constants.PreLoad.startPose.getPose());


        preLoad = LeftSemiRegionals.Constants.PreLoad.preload.build(LeftSemiRegionals.Constants.PreLoad.startPose.getPose(), constraints);

        cycle1Pickup = LeftSemiRegionals.Constants.Cycle1.pickup.build(preLoad.end(), constraints);
        cycle1Drop = LeftSemiRegionals.Constants.Cycle1.drop.build(cycle1Pickup.end(), constraints);

        cycle2Pickup = LeftSemiRegionals.Constants.Cycle2.pickup.build(cycle1Drop.end(), constraints);
        cycle2Drop = LeftSemiRegionals.Constants.Cycle2.drop.build(cycle2Pickup.end(), constraints);

        cycle3Pickup = LeftSemiRegionals.Constants.Cycle3.pickup.build(cycle2Drop.end(), constraints);
        cycle3Drop = LeftSemiRegionals.Constants.Cycle3.drop.build(cycle3Pickup.end(), constraints);

        cycle4Pickup = LeftSemiRegionals.Constants.Cycle4.pickup.build(cycle3Drop.end(), constraints);
        cycle4Drop = LeftSemiRegionals.Constants.Cycle4.drop.build(cycle4Pickup.end(), constraints);
    }

    @Override
    protected void run() {
        switch (autoState) {
            case PRELOAD:
                if (drive.isBusy()) break;
                drive.followTrajectorySequenceAsync(preLoad);
                lift.moveLow();

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
                    case 3:
                        currentPickupTrajectorySequence = cycle3Pickup;
                        currentDropTrajectorySequence = cycle3Drop;
                        currentLiftCommand = lift::moveCone3;
                        autoState = AutoState.CYCLE;
                        break;
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
                gripper.open();
                lift.moveInitial();

                drive.followTrajectorySequenceAsync(
                        //TODO make srue to update this if you add more cycles!!!!!!
                        LeftSemiRegionals.Constants.Park.getPark().build(cycle1Drop.end(), constraints));
                autoState = AutoState.END;

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
                if (waitTimer.seconds() < LeftSemiRegionals.Constants.WaitSeconds.dropDriveWait) break;
                drive.followTrajectorySequenceAsync(currentPickupTrajectorySequence);
                waitTimer.reset();
                cycleState = CycleState.PICKUP_LIFT;
                break;
            case PICKUP_LIFT:
                if (waitTimer.seconds() < LeftSemiRegionals.Constants.WaitSeconds.dropLiftWait) break;
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
                if (waitTimer.seconds() < LeftSemiRegionals.Constants.WaitSeconds.pickupLiftWait) break;
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
}
