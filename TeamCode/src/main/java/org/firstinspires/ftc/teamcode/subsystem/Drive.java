package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//import static org.firstinspires.ftc.teamcode.util.DriveConstants.MAX_ACCEL;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.MAX_ANG_ACCEL;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.MAX_ANG_VEL;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.MAX_VEL;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.MOTOR_VELO_PID;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.RUN_USING_ENCODER;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.TRACK_WIDTH;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.encoderTicksToInches;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.kA;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.kStatic;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.kV;

import static org.firstinspires.ftc.teamcode.subsystem.Drive.Constants.*;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */

public class Drive extends MecanumDrive {
    public static class Constants {
        public static Drivetrain drivetrain;
        public static class Drivetrain {
            /**
             * These are motor constants that should be listed online for your motors.
             */
            public static final double TICKS_PER_REV = 537.68984;
            public static final double MAX_RPM = 312;

            /**
             * These are physical constants that can be determined from your robot (including the track
             * width; it will be tune empirically later although a rough estimate is important). Users are
             * free to chose whichever linear distance unit they would like so long as it is consistently
             * used. The default values were selected with inches in mind. Road runner uses radians for
             * angular distances although most angular parameters are wrapped in Math.toRadians() for
             * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
             */
            public static double WHEEL_RADIUS = 1.88976; // in
            public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
            public static double TRACK_WIDTH = 12.3; // in

            public static boolean IS_FIELD_CENTRIC = true;
            public static boolean USING_FINE_CONTROL = true;



            //TODO: what does this do exactly?
            public static double LATERAL_MULTIPLIER = 1;

            public static Speed speed;
            public static class Speed {
                public static double VX_MULTIPLIER = 1.1;
                public static double VY_MULTIPLIER = 1;
                public static double OMEGA_MULTIPLIER = 1;

                public static final double NORMAL_WEIGHT = 0.6;
                public static final double FAST_WEIGHT = 1;
                public static final double SLOW_WEIGHT = 0.3;
            }
            public static Direction direction;
            public static class Direction {
                public static DcMotorSimple.Direction
                        LEFT_FRONT = DcMotorSimple.Direction.REVERSE,
                        LEFT_REAR = DcMotorSimple.Direction.REVERSE,
                        RIGHT_FRONT = DcMotorSimple.Direction.REVERSE,
                        RIGHT_REAR = DcMotorSimple.Direction.FORWARD;
            }
        }

        public static Controller controller;
        public static class Controller {
            /**
             * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
             * Set this flag to false if drive encoders are not present and an alternative localization
             * method is in use (e.g., tracking wheels).
             *
             * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
             * from DriveVelocityPIDTuner.
             */
            public static boolean RUN_USING_BUILT_IN_CONTROLLER = false;
            public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                    getMotorVelocityF(Drivetrain.MAX_RPM / 60 * Drivetrain.TICKS_PER_REV));
            /**
             * These are the feedforward parameters used to model the drive motor behavior. If you are using
             * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
             * motor encoders or have elected not to use them for velocity control, these values should be
             * empirically tuned.
             */
            public static double kV = 1.0 / rpmToVelocity(Drivetrain.MAX_RPM);
            public static double kA = 0;
            public static double kStatic = 0;

            /**
             * <p>
             * These values are used to generate the trajectories for you robot. To ensure proper operation,
             * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
             * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
             * small and gradually increase them later after everything is working. All distance units are
             * inches.
             * </p>
             * <p>
             * <b>Note from LearnRoadRunner.com:</b>
             * </p>
             * The velocity and acceleration constraints were calculated based on the following equation:<br>
             * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85<br>
             * Resulting in 52.48291908330528 in/s.<br>
             * </p>
             * <p>
             * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.<br>
             * This is capped at 85% because there are a number of variables that will prevent your bot from actually
             * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
             * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
             * max velocity. The theoretically maximum velocity is 61.74461068624151 in/s.
             * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
             * affected if it is aiming for a velocity not actually possible.
             * </p>
             * <p>
             * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
             * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
             * to degrade. As of now, it simply mirrors the velocity, resulting in 52.48291908330528 in/s/s
             *
             * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360°/s.
             * You are free to raise this on your own if you would like. It is best determined through experimentation.
             * </p>
             */
            public static double MAX_VEL       = 30; // 85% of the max for this drive would be 52
            public static double MAX_ACCEL     = 30; // 60 is about as high as this should be
            public static double MAX_ANG_VEL   = Math.toRadians(180); // 242 is about 85% of what it could do
            public static double MAX_ANG_ACCEL = Math.toRadians(180); // do maybe 242 also idk
        }

        public static Follower follower;
        public static class Follower {
            public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
            public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

            public static Pose2d ADMISSIBLE_ERROR = new Pose2d(0.5, 0.5, Math.toRadians(5.0));
            public static double TIMEOUT = 0.5;
        }

        public static double encoderTicksToInches(double ticks) {
            return Drivetrain.WHEEL_RADIUS * 2 * Math.PI * Drivetrain.GEAR_RATIO * ticks / Drivetrain.TICKS_PER_REV;
        }

        public static double rpmToVelocity(double rpm) {
            return rpm * Drivetrain.GEAR_RATIO * 2 * Math.PI * Drivetrain.WHEEL_RADIUS / 60.0;
        }

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }

    }
//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

//    public static double LATERAL_MULTIPLIER = 1;

    private double VX_WEIGHT = 1;
    private double VY_WEIGHT = 1;
    private double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(Controller.MAX_VEL, Controller.MAX_ANG_VEL, Drivetrain.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(Controller.MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;

    public Drive(OpMode opMode,boolean isUsingimu) {
        super(Controller.kV, Controller.kA, Controller.kStatic, Drivetrain.TRACK_WIDTH, Drivetrain.TRACK_WIDTH, Drivetrain.LATERAL_MULTIPLIER);
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;

        follower = new HolonomicPIDVAFollower(Follower.TRANSLATIONAL_PID, Follower.TRANSLATIONAL_PID, Follower.HEADING_PID,
                Follower.ADMISSIBLE_ERROR, Follower.TIMEOUT);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        if(isUsingimu) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }


        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (Controller.RUN_USING_BUILT_IN_CONTROLLER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Controller.RUN_USING_BUILT_IN_CONTROLLER && Controller.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Controller.MOTOR_VELO_PID);
        }

        leftFront.setDirection(Drivetrain.Direction.LEFT_FRONT);
        rightFront.setDirection(Drivetrain.Direction.RIGHT_FRONT);
        rightRear.setDirection(Drivetrain.Direction.RIGHT_REAR);
        leftRear.setDirection(Drivetrain.Direction.LEFT_REAR);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, Follower.HEADING_PID);
    }
    public Drive (OpMode opMode) {
        this(opMode,false);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                Controller.MAX_ANG_VEL, Controller.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    /**
     * Square magnitude of number while keeping the sign.
     */
    private double squareInput(double input) {
        return input * Math.abs(input);
    }

    /**
     *
     *
     * @param gyroAngle angle the bot is currently facing relative to start (radians)
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        driveRobotCentric(
                (strafeSpeed * Math.cos(gyroAngle) - forwardSpeed * Math.sin(gyroAngle)),
                (strafeSpeed * Math.sin(gyroAngle) + forwardSpeed * Math.cos(gyroAngle)),
                turnSpeed);
    }
    
    private void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        setWeightedDrivePower(
                new Pose2d(
                        forwardSpeed,
                        -strafeSpeed,
                        -turnSpeed
                )
        );
    }

    public void manualControl() {
        double strafeSpeed;
        double forwardSpeed;
        double turnSpeed;

        if (Drivetrain.USING_FINE_CONTROL) {
            strafeSpeed = squareInput(gamepad1.left_stick_x);
            forwardSpeed = squareInput(-gamepad1.left_stick_y);
            turnSpeed = squareInput(gamepad1.right_stick_x);
        } else {
            strafeSpeed = gamepad1.left_stick_x;
            forwardSpeed = -gamepad1.left_stick_y;
            turnSpeed = gamepad1.right_stick_x;
        }

        if (gamepad1.left_bumper) {
            VX_WEIGHT = Drivetrain.Speed.SLOW_WEIGHT * Drivetrain.Speed.VX_MULTIPLIER;
            VY_WEIGHT = Drivetrain.Speed.SLOW_WEIGHT * Drivetrain.Speed.VY_MULTIPLIER;
            OMEGA_WEIGHT = Drivetrain.Speed.SLOW_WEIGHT * Drivetrain.Speed.OMEGA_MULTIPLIER;
        }
        else if (gamepad1.right_bumper) {
            VX_WEIGHT = Drivetrain.Speed.FAST_WEIGHT * Drivetrain.Speed.VX_MULTIPLIER;
            VY_WEIGHT = Drivetrain.Speed.FAST_WEIGHT * Drivetrain.Speed.VY_MULTIPLIER;
            OMEGA_WEIGHT = Drivetrain.Speed.FAST_WEIGHT * Drivetrain.Speed.OMEGA_MULTIPLIER;
        }
        else {
            VX_WEIGHT = Drivetrain.Speed.NORMAL_WEIGHT * Drivetrain.Speed.VX_MULTIPLIER;
            VY_WEIGHT = Drivetrain.Speed.NORMAL_WEIGHT * Drivetrain.Speed.VY_MULTIPLIER;
            OMEGA_WEIGHT = Drivetrain.Speed.NORMAL_WEIGHT * Drivetrain.Speed.OMEGA_MULTIPLIER;
        }


        if (Drivetrain.IS_FIELD_CENTRIC)
            driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, -getRawExternalHeading());
        else
            driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);

    }

    public void teleOpControlTest() {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}
