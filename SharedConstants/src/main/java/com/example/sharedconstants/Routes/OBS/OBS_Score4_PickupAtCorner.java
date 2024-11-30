package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_135_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_340_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_NINE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_APPROACH_ALLIANCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_APPROACH_AUDIENCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_PICKUP_ALLIANCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_PICKUP_AUDIENCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_STAGING_AUDIENCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_REDO;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.MOVE_PRELOAD_SPECIMEN_TO_CW_HOME;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;

public class OBS_Score4_PickupAtCorner extends Routes {

    private static final double OBS_FAST_VELOCITY_OVERRIDE = 55;
    private static final double OBS_FAST_ACCELERATION_OVERRIDE = 55;
    private static final double OBS_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double OBS_SLOW_VELOCITY_OVERRIDE = 10;
    private static final double OBS_SLOW_ACCELERATION_OVERRIDE = 10;
    private static final double OBS_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);


    public OBS_Score4_PickupAtCorner(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        SetupConstraints();
        scoreObservationPreload(CHAMBER_SLOT_ONE_REDO);
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        pushThirdNeutralSpecimen();
        pickupSpecimenFromCornerAllianceWall();
        scoreOnHighChamberFromCorner(CHAMBER_SLOT_THREE_REDO);
        pickupSpecimenFromCornerAudienceWall();
        scoreOnHighChamberFromCornerAudienceWall(CHAMBER_SLOT_FIVE_REDO);
        pickupSpecimenFromCornerAudienceWall();
        scoreOnHighChamberFromCornerAudienceWall(CHAMBER_SLOT_SEVEN_REDO);
        pickupSpecimenFromCornerAudienceWall();
        scoreOnHighChamberFromCornerAudienceWall(CHAMBER_SLOT_NINE_REDO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }


    public static VelConstraint obsFastVelocity;
    public static AccelConstraint obsFastAcceleration;

    public static VelConstraint obsSlowVelocity;
    public static AccelConstraint obsSlowAcceleration;

    public void SetupConstraints() {
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

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToSplineHeading(chamberSlot.plus(new Twist2d(new Vector2d(-10,0), 0)), chamberSlot.heading.toDouble(), obsFastVelocity)
                .splineToConstantHeading(PoseToVector(chamberSlot), chamberSlot.heading.toDouble(), obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.15);
    }

    private void pickupSpecimenFromCornerAllianceWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_CORNER_APPROACH_ALLIANCE_WALL), ANGLE_TOWARD_RED, obsFastVelocity)
                .stopAndAdd(new NullAction())
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(OBS_CORNER_PICKUP_ALLIANCE_WALL), ANGLE_TOWARD_RED, obsSlowVelocity, obsSlowAcceleration)
                .waitSeconds(.12);
    }

    public void pushFirstNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RIGHT_OF_CHAMBER_REDO), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_ONE_REDO),ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .splineTo(PoseToVector(OBS_DELIVER_SPIKE_ONE_REDO),ANGLE_TOWARD_RED, obsFastVelocity)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_ONE_REDO),ANGLE_TOWARD_BLUE, obsFastVelocity);
    }

    public void pushSecondNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_TWO_REDO), ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_TWO_REDO),ANGLE_TOWARD_RED, obsFastVelocity)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_TWO_REDO), ANGLE_TOWARD_BLUE, obsFastVelocity);
    }

    private void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_THREE_REDO), ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_THREE_REDO),ANGLE_TOWARD_RED, obsFastVelocity);
    }


    public void scoreOnHighChamberFromCorner(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToLinearHeading(chamberSlot.plus(new Twist2d(new Vector2d(-5,0),0)), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .stopAndAdd(new NullAction())
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void scoreOnHighChamberFromCornerAudienceWall(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_135_DEGREES)
                .splineToLinearHeading(chamberSlot.plus(new Twist2d(new Vector2d(-5,0),0)), ANGLE_135_DEGREES, obsFastVelocity)
                .stopAndAdd(new NullAction())
                .setReversed(false)
                .splineToLinearHeading(chamberSlot, ANGLE_TOWARD_BLUE, obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void pickupSpecimenFromCornerAudienceWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(4, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_CORNER_STAGING_AUDIENCE_WALL, ANGLE_315_DEGREES, obsFastVelocity)
                .setTangent(ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_CORNER_PICKUP_AUDIENCE_WALL, ANGLE_TOWARD_OBSERVATION, obsSlowVelocity)
                .waitSeconds(.12);
    }

}

