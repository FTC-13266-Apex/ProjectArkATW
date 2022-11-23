package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmode.tuner.AutomaticFeedforwardTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.BackAndForth;
import org.firstinspires.ftc.teamcode.opmode.tuner.DriveVelocityPIDTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.MaxAngularVeloTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.MaxVelocityTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.MotorDirectionDebugger;
import org.firstinspires.ftc.teamcode.opmode.tuner.SplineTest;
import org.firstinspires.ftc.teamcode.opmode.tuner.StrafeTest;
import org.firstinspires.ftc.teamcode.opmode.tuner.StraightTest;
import org.firstinspires.ftc.teamcode.opmode.tuner.TrackWidthTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.TrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.TrackingWheelLateralDistanceTuner;
import org.firstinspires.ftc.teamcode.opmode.tuner.TurnTest;
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
    public static RoadRunnerConfiguration drive;




    public static class RoadRunnerConfiguration {
        public static Drive.Constants drivetrain;
        public static StandardTrackingWheelLocalizer.Constants deadWheels;
        public static Tuners tuners;
        public static TrajectorySequenceRunner.Constants trajectorySequenceRunner;

        public static class Tuners {
            AutomaticFeedforwardTuner.Constants automaticFeedforwardTuner;
            BackAndForth.Constants backAndForth;
            DriveVelocityPIDTuner.Constants driveVelocityPIDTuner;
            ManualFeedforwardTuner.Constants manualFeedforwardTuner;
            MaxAngularVeloTuner.Constants maxAngularVeloTuner;
            MaxVelocityTuner.Constants maxVelocityTuner;
            MotorDirectionDebugger.Constants motorDirectionDebugger;
            SplineTest.Constants splineTest;
            StrafeTest.Constants strafeTest;
            StraightTest.Constants straightTest;
            TrackingWheelForwardOffsetTuner.Constants trackingWheelForwardOffsetTuner;
            TrackingWheelLateralDistanceTuner.Constants trackingWheelLateralDistanceTuner;
            TrackWidthTuner.Constants trackWidthTuner;
            TurnTest.Constants turnTest;
        }
    }
}
