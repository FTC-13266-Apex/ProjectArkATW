package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.command.FlipConeWithGamepad;
import org.firstinspires.ftc.teamcode.command.Sensor;
import org.firstinspires.ftc.teamcode.opmode.auto.LeftSide;
import org.firstinspires.ftc.teamcode.opmode.auto.sample.right.Container;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.AutomaticFeedforwardTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.BackAndForth;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.DriveVelocityPIDTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.MaxAngularVeloTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.MaxVelocityTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.MotorDirectionDebugger;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.ServoPositionTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.SplineTest;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.StrafeTest;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.StraightTest;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.TrackWidthTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.TrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.TrackingWheelLateralDistanceTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.roadrunner.TurnTest;
import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

@Config
public class Constants {
    public static Lift.Constants lift;
    public static Gripper.Constants gripper;
    public static Vision.Constants vision;
    public static ConeFlipper.Constants coneFlipper;




    public static Autos autos;
    public static class Autos {
        public static LeftSide.Constants rightSideOnly;
        public static Container.Constants experimentalRight;
    }

    public static Commands commands;
    public static class Commands {
        public static Sensor.Constants sensor;
        public static FlipConeWithGamepad.Constants flipConeWIthGamepad;
    }

    public static RoadRunnerConfiguration drive;
    public static class RoadRunnerConfiguration {
        public static Drive.Constants drivetrain;
        public static StandardTrackingWheelLocalizer.Constants deadWheels;
        public static TrajectorySequenceRunner.Constants trajectorySequenceRunner;
    }

    public static Tuners tuners;
    public static class Tuners {
        public static AutomaticFeedforwardTuner.Constants automaticFeedforwardTuner;
        public static BackAndForth.Constants backAndForth;
        public static DriveVelocityPIDTuner.Constants driveVelocityPIDTuner;
        public static ManualFeedforwardTuner.Constants manualFeedforwardTuner;
        public static MaxAngularVeloTuner.Constants maxAngularVeloTuner;
        public static MaxVelocityTuner.Constants maxVelocityTuner;
        public static MotorDirectionDebugger.Constants motorDirectionDebugger;
        public static SplineTest.Constants splineTest;
        public static StrafeTest.Constants strafeTest;
        public static StraightTest.Constants straightTest;
        public static TrackingWheelForwardOffsetTuner.Constants trackingWheelForwardOffsetTuner;
        public static TrackingWheelLateralDistanceTuner.Constants trackingWheelLateralDistanceTuner;
        public static TrackWidthTuner.Constants trackWidthTuner;
        public static TurnTest.Constants turnTest;
        public static ServoPositionTuner.Constants servoPositionTuner;
    }

    public static Utilities utilities;
    public static class Utilities {
        public static TrajectorySequenceRunner.Constants trajectorySequenceRunner;
        public static StandardTrackingWheelLocalizer.Constants standardTrackingWheelLocalizer;
    }
}
