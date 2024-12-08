package com.example.sharedconstants.Routes.OBS;

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
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_TIP_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE;
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
    public double SLOW_VELOCITY_OVERRIDE = 10;
    public double SLOW_ACCELERATION_OVERRIDE = 10;
    public double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    public double NORMAL_VELOCITY_OVERRIDE = 28;
    public double NORMAL_ACCELERATION_OVERRIDE = 28;
    public double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    public double FAST_VELOCITY_OVERRIDE = 30;
    public double FAST_ACCELERATION_OVERRIDE = 30;
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
        pickupSpecimenFromTriangleComingFromThirdSpike();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_ONE_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_TWO_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FOUR_REDO);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(2, robotAdapter.getAction(MOVE_PRELOAD_SPECIMEN_TO_CW_HOME))
                .splineToLinearHeading(chamberSlot.plus(new Twist2d(new Vector2d(-10,0), 0)), chamberSlot.heading.toDouble(), normalVelocity)
                .stopAndAdd(new NullAction())
                .splineToConstantHeading(PoseToVector(chamberSlot), chamberSlot.heading.toDouble(), slowVelocity)
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .waitSeconds(.15);
    }

    public void pickupGroundSampleOne() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(3 , robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .splineToLinearHeading(Pose2d.exp(OBS_SPIKE_ONE.minus(new Pose2d(new Vector2d(- 1.5 , 22),0))) , ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(.8); // give some time to grab it from the ground and do the transfer
    }

    private void pickupGroundSampleTwo() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(1 , robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .afterDisp(2 , robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE)) // dump the sample while pulling in the next one
                .strafeToLinearHeading(PoseToVector(OBS_SPIKE_TWO).minus(new Vector2d(- 3.5 , 22)) , ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(.8);
    }

    private void pickupGroundSampleThree() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(1 , robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .afterDisp(2 , robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE))
                .strafeToLinearHeading(PoseToVector(OBS_SPIKE_THREE).minus(new Vector2d(4 , 22)) , Math.toRadians(60))
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(.8);
    }


    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-3)), chamberSlot.heading.toDouble(), normalVelocity)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, slowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void pickupSpecimenFromTriangleComingFromThirdSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
//                .turnTo(Math.toRadians(95))
                .setTangent(ANGLE_TOWARD_NET)
                .afterDisp(0 , robotAdapter.getAction(DUMP_SAMPLE_IN_OBSERVATION_ZONE))
                .afterDisp(5 , robotAdapter.getAction(GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_TRIANGLE_TIP_APPROACH, ANGLE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_TIP_PICKUP , ANGLE_TOWARD_RED , slowVelocity);
    }


    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(3 , robotAdapter.getAction(GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_TIP_PICKUP).plus(new Vector2d(0,3)), ANGLE_TOWARD_RED, normalVelocity)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_TIP_PICKUP , ANGLE_TOWARD_RED , slowVelocity);
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
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, fastVelocity, fastAcceleration);
    }
}


