package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class TrajectorySequenceConstraints {
    double baseVelConstraint;
    double baseAccelConstraint;
    double baseTurnConstraintMaxAngVel;
    double baseTurnConstraintMaxAngAccel;

    public TrajectorySequenceConstraints(
            double baseVelConstraint,
            double baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;
        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;
    }
}
