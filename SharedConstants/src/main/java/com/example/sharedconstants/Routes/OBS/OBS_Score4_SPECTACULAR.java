package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public class OBS_Score4_SPECTACULAR extends OBS_Score_1_Specimen_Preload {

    private static final double OBS_FAST_VELOCITY_OVERRIDE = 27.5;
    private static final double OBS_FAST_ACCELERATION_OVERRIDE = 27.5;
    private static final double OBS_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double OBS_VELOCITY_OVERRIDE = 27.5;
    private static final double OBS_ACCELERATION_OVERRIDE = 27.5;
    private static final double OBS_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double OBS_SLOW_VELOCITY_OVERRIDE = 20;
    private static final double OBS_SLOW_ACCELERATION_OVERRIDE = 20;
    private static final double OBS_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static VelConstraint obsFastVelocity;
    public static AccelConstraint obsFastAcceleration;

    public static VelConstraint obsVelocity;
    public static AccelConstraint obsAcceleration;

    public static VelConstraint obsSlowVelocity;
    public static AccelConstraint obsSlowAcceleration;

    public OBS_Score4_SPECTACULAR(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        SetupConstraints();
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        pickupSpecimenFromTriangleComingFromSecondSpike();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_SEVEN);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }


    public void pushFirstNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RIGHT_OF_CHAMBER), ANGLE_TOWARD_BLUE, obsFastVelocity, obsFastAcceleration)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_ONE),ANGLE_TOWARD_OBSERVATION, obsFastVelocity, obsFastAcceleration)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_ONE),ANGLE_TOWARD_RED , obsFastVelocity, obsFastAcceleration)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_ONE), ANGLE_TOWARD_NET, obsFastVelocity, obsFastAcceleration);

    }

    public void pushSecondNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_TWO), Math.toRadians(350), obsFastVelocity, obsFastAcceleration)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_TWO), ANGLE_TOWARD_RED, obsFastVelocity, obsFastAcceleration)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_TWO), ANGLE_TOWARD_RED, obsFastVelocity, obsFastAcceleration);
    }

    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_115_DEGREES)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_115_DEGREES, obsFastVelocity, obsFastAcceleration)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(6, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_APPROACH), ANGLE_TOWARD_RED, obsVelocity, obsAcceleration)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, obsSlowVelocity, obsSlowAcceleration);
    }

    public void pickupSpecimenFromTriangleComingFromSecondSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(5, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_APPROACH), ANGLE_TOWARD_RED, obsVelocity, obsAcceleration)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, obsSlowVelocity, obsSlowAcceleration);
    }


    private void driveToPark() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, obsVelocity, obsAcceleration);

    }



    public void SetupConstraints(){
        obsVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(OBS_VELOCITY_OVERRIDE),
                new AngularVelConstraint(OBS_ANGULAR_VELOCITY_OVERRIDE)
        ));
        obsAcceleration = new ProfileAccelConstraint(-OBS_ACCELERATION_OVERRIDE, OBS_ACCELERATION_OVERRIDE);

        obsFastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(OBS_FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(OBS_FAST_ANGULAR_VELOCITY_OVERRIDE)
        ));
        obsFastAcceleration = new ProfileAccelConstraint(-OBS_FAST_ACCELERATION_OVERRIDE, OBS_FAST_ACCELERATION_OVERRIDE);

        obsSlowVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(OBS_SLOW_VELOCITY_OVERRIDE),
                new AngularVelConstraint(OBS_SLOW_ANGULAR_VELOCITY_OVERRIDE)
        ));
        obsSlowAcceleration = new ProfileAccelConstraint(-OBS_SLOW_ACCELERATION_OVERRIDE, OBS_SLOW_ACCELERATION_OVERRIDE);
    }
}

