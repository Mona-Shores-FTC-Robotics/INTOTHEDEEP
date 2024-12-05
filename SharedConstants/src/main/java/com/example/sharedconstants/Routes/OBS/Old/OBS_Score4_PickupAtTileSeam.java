package com.example.sharedconstants.Routes.OBS.Old;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH_TILE_SEAM;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP_TILE_SEAM;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_REDO;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.MOVE_PRELOAD_SPECIMEN_TO_CW_HOME;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
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

public class OBS_Score4_PickupAtTileSeam extends Routes {

    public OBS_Score4_PickupAtTileSeam(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        SetupConstraints();
        scoreObservationPreload(CHAMBER_SLOT_ONE_REDO);
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        pushThirdNeutralSpecimen();
        pickupSpecimenFromTileSeam();
        scoreOnHighChamberFromCorner(CHAMBER_SLOT_THREE_REDO);
        pickupSpecimenFromTileSeam();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE_REDO);
        pickupSpecimenFromTileSeam();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_SEVEN_REDO);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }


    private static final double OBS_FAST_VELOCITY_OVERRIDE = 55;
    private static final double OBS_FAST_ACCELERATION_OVERRIDE = 55;
    private static final double OBS_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double OBS_SLOW_VELOCITY_OVERRIDE = 7;
    private static final double OBS_SLOW_ACCELERATION_OVERRIDE = 7;
    private static final double OBS_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

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
                .waitSeconds(.01)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_THREE_REDO),ANGLE_TOWARD_RED, obsFastVelocity);
    }


    public void scoreOnHighChamberFromCorner(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-5)), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .waitSeconds(.01)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-5)), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .waitSeconds(.01)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }


    public void pickupSpecimenFromTileSeam() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(1, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_TRIANGLE_APPROACH_TILE_SEAM, ANGLE_TOWARD_RED, obsFastVelocity)
                .waitSeconds(.01)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP_TILE_SEAM, ANGLE_TOWARD_RED, obsSlowVelocity)
                .waitSeconds(.15);
    }

    private void driveToPark() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_PICKUP), ANGLE_TOWARD_RED, obsFastVelocity);
    }
}

