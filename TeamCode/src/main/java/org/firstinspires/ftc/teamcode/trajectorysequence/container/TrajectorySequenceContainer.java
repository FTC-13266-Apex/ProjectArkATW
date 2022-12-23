package org.firstinspires.ftc.teamcode.trajectorysequence.container;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectorySequenceContainer {
    private final PathSegment[] pathSegments;
    private TrajectorySequence trajectorySequence;
    public TrajectorySequenceContainer (PathSegment... pathSegments) {
        this.pathSegments = pathSegments;
    }

    public TrajectorySequence build(TrajectorySequenceBuilder trajectorySequenceBuilder) {
        for (PathSegment pathSegment : pathSegments) {
            if (pathSegment.getClass() == LineTo.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineTo((LineTo) pathSegment); // cast as a LineTo

            if (pathSegment.getClass() == LineToConstantHeading.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineToConstantHeading((LineToConstantHeading) pathSegment);

            if (pathSegment.getClass() == LineToLinearHeading.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineToLinearHeading(((LineToLinearHeading) pathSegment));

            if (pathSegment.getClass() == LineToSplineHeading.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.lineToSplineHeading(((LineToSplineHeading) pathSegment));

            if (pathSegment.getClass() == SplineTo.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineTo(((SplineTo) pathSegment));

            if (pathSegment.getClass() == SplineToConstantHeading.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineToConstantHeading(((SplineToConstantHeading) pathSegment));

            if (pathSegment.getClass() == SplineToLinearHeading.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineToLinearHeading(((SplineToLinearHeading) pathSegment));

            if (pathSegment.getClass() == SplineToSplineHeading.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.splineToSplineHeading(((SplineToSplineHeading) pathSegment));

            if (pathSegment.getClass() == Forward.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.forward(((Forward) pathSegment).distance);

            if (pathSegment.getClass() == Back.class)
                trajectorySequenceBuilder = trajectorySequenceBuilder.back(((Back) pathSegment).distance);

        }
        trajectorySequence = trajectorySequenceBuilder.build();
        return trajectorySequence;
    }

    public TrajectorySequence build(
            Pose2d startPose,
            Double startTangent,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        return this.build(new TrajectorySequenceBuilder(
                startPose,
                startTangent,
                baseVelConstraint,
                baseAccelConstraint,
                baseTurnConstraintMaxAngVel,
                baseTurnConstraintMaxAngAccel
        ));
    }

    public TrajectorySequence build(
            Pose2d startPose,
            double baseVelConstraint,
            double baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        return this.build(new TrajectorySequenceBuilder(
                startPose,
                (v, pose2d, pose2d1, pose2d2) -> baseVelConstraint,
                (v, pose2d, pose2d1, pose2d2) -> baseAccelConstraint,
                baseTurnConstraintMaxAngVel,
                baseTurnConstraintMaxAngAccel
        ));
    }

    public TrajectorySequence build(
            Pose2d startPose,
            TrajectorySequenceConstraints constraints
    ) {
        return this.build(
                startPose,
                constraints.baseVelConstraint,
                constraints.baseAccelConstraint,
                constraints.baseTurnConstraintMaxAngVel,
                constraints.baseTurnConstraintMaxAngAccel
        );
    }

    public Pose2d end() {
        return trajectorySequence.end();
    }
}
