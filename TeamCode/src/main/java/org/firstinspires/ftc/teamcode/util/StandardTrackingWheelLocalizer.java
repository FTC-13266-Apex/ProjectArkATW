package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static class Constants {
        public static double TICKS_PER_REV = 8192;
        public static double WHEEL_RADIUS = 0.6889764; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

        public static double LATERAL_DISTANCE = 10.386; // in; distance between the left and right wheels
        public static double FORWARD_OFFSET = -5.03297; // in; offset of the lateral wheel

        public static double X_MULTIPLIER = 1; // Multiplier in the X direction
        public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

        public static Direction direction;
        public static class Direction {
            public static Encoder.Direction
                LEFT = Encoder.Direction.REVERSE,
                RIGHT = Encoder.Direction.REVERSE,
                BACK = Encoder.Direction.REVERSE;
        }
    }


    private Encoder leftEncoder, rightEncoder, rearEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, Constants.LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -Constants.LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(Constants.FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        // 0 - rightRear
        // 1- leftFront
        // 2 - leftRear
        // 3 - rightFront

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear")); // 0
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront")); // 1
        rearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear")); // 2

        leftEncoder.setDirection(Constants.Direction.LEFT);
        rightEncoder.setDirection(Constants.Direction.RIGHT);
        rearEncoder.setDirection(Constants.Direction.BACK);
    }

    public static double encoderTicksToInches(double ticks) {
        return Constants.WHEEL_RADIUS * 2 * Math.PI * Constants.GEAR_RATIO * ticks / Constants.TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * Constants.X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * Constants.X_MULTIPLIER),
                encoderTicksToInches(rearEncoder.getCurrentPosition() * Constants.Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()* Constants.X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()* Constants.X_MULTIPLIER),
                encoderTicksToInches(rearEncoder.getCorrectedVelocity() * Constants.Y_MULTIPLIER)
        );
    }
}
