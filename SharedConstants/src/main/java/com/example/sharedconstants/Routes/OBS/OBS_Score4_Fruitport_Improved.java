package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH_TILE_SEAM;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP_TILE_SEAM;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;
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
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score_1_Specimen_Preload;

import java.util.Arrays;

public class OBS_Score4_Fruitport_Improved extends OBS_Score_1_Specimen_Preload {

    private static final double OBS_FAST_VELOCITY_OVERRIDE = 50;
    private static final double OBS_FAST_ACCELERATION_OVERRIDE = 50;
    private static final double OBS_FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double OBS_VELOCITY_OVERRIDE = 40;
    private static final double OBS_ACCELERATION_OVERRIDE = 40;
    private static final double OBS_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    private static final double OBS_HAIRPIN_VELOCITY = 30;
    private static final double OBS_HAIRPIN_ACCELERATION= 30;
    private static final double OBS_HAIRPIN_ANGULAR= Math.toRadians(360);

    private static final double OBS_SLOW_VELOCITY_OVERRIDE = 10;
    private static final double OBS_SLOW_ACCELERATION_OVERRIDE = 25;
    private static final double OBS_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static VelConstraint obsFastVelocity;
    public static AccelConstraint obsFastAcceleration;

    public static VelConstraint obsVelocity;
    public static AccelConstraint obsAcceleration;

    public static VelConstraint obsSlowVelocity;
    public static AccelConstraint obsSlowAcceleration;

    public static VelConstraint hairpinVelocity;
    public static AccelConstraint hairpinAcceleration;

    public OBS_Score4_Fruitport_Improved(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        SetupConstraints();
        scoreObservationPreload(CHAMBER_SLOT_ONE_REDO);
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        driveToTriangleApproachFromSecondSpike();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE_REDO);
        driveToTriangleApproachFromChamber();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE_REDO);
        driveToTriangleApproachFromChamber();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_SEVEN_REDO);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToLinearHeading(chamberSlot.plus(new Twist2d(new Vector2d(-7,0), 0)), chamberSlot.heading.toDouble(), obsVelocity)
                .stopAndAdd(new NullAction())
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
                .lineToY(OBS_DELIVER_SPIKE_ONE_REDO.position.y, obsFastVelocity)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_ONE_REDO),ANGLE_TOWARD_BLUE, obsFastVelocity);
    }

    public void pushSecondNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_TWO_REDO), ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .lineToY(OBS_DELIVER_SPIKE_TWO_REDO.position.y, obsFastVelocity);
    }

    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-7)), chamberSlot.heading.toDouble(), obsVelocity)
                .stopAndAdd(new NullAction())
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity, obsSlowAcceleration)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void driveToTriangleApproachFromChamber()
    {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(6, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_TIP_APPROACH), ANGLE_TOWARD_RED, obsVelocity, obsAcceleration)
                .stopAndAdd(new NullAction());
    }

    public void driveToTriangleApproachFromSecondSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder

                .afterDisp(5, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .setTangent(ANGLE_TOWARD_NET)
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_TIP_APPROACH), ANGLE_TOWARD_RED, hairpinVelocity, hairpinAcceleration)
                .stopAndAdd(new NullAction());
    }

    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_TIP_PICKUP, ANGLE_TOWARD_RED, obsSlowVelocity, obsSlowAcceleration)
                .waitSeconds(.2);
    }

    private void driveToPark() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
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

        hairpinVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(OBS_HAIRPIN_VELOCITY),
                new AngularVelConstraint(OBS_HAIRPIN_ANGULAR)
        ));
        hairpinAcceleration = new ProfileAccelConstraint(-OBS_HAIRPIN_ACCELERATION, OBS_HAIRPIN_ACCELERATION);
    }
}

