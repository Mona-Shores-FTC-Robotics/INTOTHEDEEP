package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_APPROACH_ALLIANCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_PICKUP_ALLIANCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DISABLE_PRELOAD_MODE;
import static com.example.sharedconstants.RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL;
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
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;

public class OBS_Score5_Specimen_Preload_Ground_Pickup extends Routes {

    // Velocity and acceleration overrides
    public double SLOW_VELOCITY_OVERRIDE = 15;
    public double SLOW_ACCELERATION_OVERRIDE = 25;
    public double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public double NORMAL_VELOCITY_OVERRIDE = 28;
    public double NORMAL_ACCELERATION_OVERRIDE = 28;
    public double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    public double FAST_VELOCITY_OVERRIDE = 60;
    public double FAST_ACCELERATION_OVERRIDE = 60;
    public double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    // Shared constraints for all routes
    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint normalVelocity;
    public static AccelConstraint normalAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;

    public OBS_Score5_Specimen_Preload_Ground_Pickup(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        setupConstraints();
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE);
        scoreObservationPreload(CHAMBER_SLOT_ONE_REDO);
        pickupGroundSampleOne();
        pickupGroundSampleTwo();
        pickupGroundSampleThree();
        pickupSpecimenFromFieldCornerComingFromThirdSpike();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_TWO_REDO);  //make this different maybe?
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FOUR_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE_REDO);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToLinearHeading(chamberSlot.plus(new Twist2d(new Vector2d(-10,0), 0)), chamberSlot.heading.toDouble(), fastVelocity, fastAcceleration)
                .stopAndAdd(new NullAction())
                .splineToConstantHeading(PoseToVector(chamberSlot), chamberSlot.heading.toDouble(), slowVelocity)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.15);
    }
    public void pickupGroundSampleOne() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(15,robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(
                        new Pose2d(PoseToVector(OBS_SPIKE_ONE).minus(new Vector2d(- 1.5 , 23)), ANGLE_TOWARD_BLUE),
                        ANGLE_TOWARD_OBSERVATION, normalVelocity)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(1.5); // give some time to grab it from the ground and do the transfer
    }

    private void pickupGroundSampleTwo() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterTime(.8 , robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .afterTime( 1.4, robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE)) // dump the sample while pulling in the next one
                .strafeToLinearHeading(PoseToVector(OBS_SPIKE_TWO).minus(new Vector2d(- 4.70 , 23)) , ANGLE_TOWARD_BLUE, slowVelocity)
                .waitSeconds(.3)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(1.2);
    }

    private void pickupGroundSampleThree() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterTime(.4, robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE))
                .afterTime(.6, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .afterTime(.9, robotAdapter.getAction(GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .strafeToLinearHeading(PoseToVector(OBS_SPIKE_THREE).minus(new Vector2d(4 , 23)) , Math.toRadians(59), slowVelocity)
                .waitSeconds(.3)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(1.2);
    }

    public void pickupSpecimenFromFieldCornerComingFromThirdSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterTime(1.2 , robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE))
                .afterTime(1.5, robotAdapter.getAction(FLIP_UP_AND_RETRACT))
                .splineToLinearHeading(OBS_CORNER_APPROACH_ALLIANCE_WALL, ANGLE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(OBS_CORNER_PICKUP_ALLIANCE_WALL , ANGLE_TOWARD_RED , slowVelocity);
    }

    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_115_DEGREES)
                .strafeToLinearHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-5.5)), chamberSlot.heading.toDouble(), fastVelocity, fastAcceleration)
                .strafeToLinearHeading(PoseToVector(chamberSlot), chamberSlot.heading.toDouble(), slowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER))
                .waitSeconds(.15);
    }

    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(3 , robotAdapter.getAction(GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .strafeToLinearHeading(PoseToVector(OBS_TRIANGLE_TIP_APPROACH), ANGLE_TOWARD_BLUE, fastVelocity, fastAcceleration)
                .waitSeconds(.1)
                .setReversed(true)
                .strafeToLinearHeading(PoseToVector(OBS_TRIANGLE_TIP_PICKUP).plus(new Vector2d(0,2)), ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .waitSeconds(.1);
    }

    private void setupConstraints() {
        slowVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SLOW_VELOCITY_OVERRIDE) ,
                new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)
        ));
        slowAcceleration = new ProfileAccelConstraint(- SLOW_ACCELERATION_OVERRIDE , SLOW_ACCELERATION_OVERRIDE);

        normalVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(NORMAL_VELOCITY_OVERRIDE) ,
                new AngularVelConstraint(NORMAL_ANGULAR_VELOCITY_OVERRIDE)
        ));
        normalAcceleration = new ProfileAccelConstraint(- NORMAL_ACCELERATION_OVERRIDE , NORMAL_ACCELERATION_OVERRIDE);

        fastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(FAST_VELOCITY_OVERRIDE) ,
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)
        ));
        fastAcceleration = new ProfileAccelConstraint(- FAST_ACCELERATION_OVERRIDE , FAST_ACCELERATION_OVERRIDE);
    }

    private void driveToPark() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .strafeToLinearHeading(PoseToVector(OBS_TRIANGLE_PICKUP), ANGLE_TOWARD_BLUE, fastVelocity, fastAcceleration);
    }
}


