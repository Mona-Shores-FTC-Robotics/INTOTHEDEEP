package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_REDO;
import static com.example.sharedconstants.RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.MOVE_PRELOAD_SPECIMEN_TO_CW_HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND;

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

public class OBS_Score5_Push_3 extends OBS_Score_1_Specimen_Preload {

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

    public OBS_Score5_Push_3(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        SetupConstraints();
        scoreObservationPreload(CHAMBER_SLOT_ONE_REDO);
//        pickupGroundSampleOne();
//        pickupGroundSampleTwo();
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        pushThirdNeutralSpecimen();
        driveToTriangleApproachFromSecondSpike();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_TWO_REDO);
        driveToTriangleApproachFromChamber();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE_REDO);
        driveToTriangleApproachFromChamber();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FOUR_REDO);
        driveToTriangleApproachFromChamber();
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE_REDO);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToLinearHeading(chamberSlot.plus(new Twist2d(new Vector2d(-10,0), 0)), chamberSlot.heading.toDouble(), obsVelocity)
                .stopAndAdd(new NullAction())
                .splineToConstantHeading(PoseToVector(chamberSlot), chamberSlot.heading.toDouble(), obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.15);
    }

    public void pickupGroundSampleOne() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterTime(2.1 , robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .afterTime(2.5, robotAdapter.getAction(PICKUP_FROM_GROUND))
                .splineToLinearHeading(
                        new Pose2d(PoseToVector(OBS_SPIKE_ONE).minus(new Vector2d(- 1.5 , 23)), ANGLE_TOWARD_BLUE),
                        ANGLE_TOWARD_OBSERVATION, fastVelocity)
                .waitSeconds(.64); // give some time to grab it from the ground and do the transfer
    }

    private void pickupGroundSampleTwo() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterTime( 1.3, robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE)) // dump the sample while pulling in the next one
                .afterTime(.8 , robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .strafeToLinearHeading(PoseToVector(OBS_SPIKE_TWO).minus(new Vector2d(- 4.70 , 23)) , ANGLE_TOWARD_BLUE, obsSlowVelocity, obsSlowAcceleration)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(1.1);
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

    public void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_115_DEGREES)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_THREE_REDO), ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .lineToY(OBS_DELIVER_SPIKE_THREE_REDO.position.y, obsFastVelocity);
    }


    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .strafeToLinearHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-7)), chamberSlot.heading.toDouble(), obsVelocity)
                .stopAndAdd(new NullAction())
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity, obsSlowAcceleration)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void driveToTriangleApproachFromChamber()
    {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(6, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .strafeToLinearHeading(PoseToVector(OBS_TRIANGLE_TIP_APPROACH), ANGLE_TOWARD_BLUE, obsVelocity, obsAcceleration)
                .stopAndAdd(new NullAction());
    }

    public void driveToTriangleApproachFromSecondSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(5, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .setTangent(ANGLE_TOWARD_NET)
                .strafeToLinearHeading(PoseToVector(OBS_TRIANGLE_TIP_APPROACH), ANGLE_TOWARD_BLUE, fastVelocity, fastAcceleration)
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

