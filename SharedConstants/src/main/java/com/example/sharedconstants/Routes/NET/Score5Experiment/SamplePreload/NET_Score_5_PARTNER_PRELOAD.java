package com.example.sharedconstants.Routes.NET.Score5Experiment.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_160_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.PARTNER_PRELOAD_LEFT_BEHIND;
import static com.example.sharedconstants.FieldConstants.SAMPLE_LENGTH;
import static com.example.sharedconstants.RobotAdapter.ActionType.CONDITIONAL_PICKUP;
import static com.example.sharedconstants.RobotAdapter.ActionType.CONDITIONAL_TRANSFER;
import static com.example.sharedconstants.RobotAdapter.ActionType.DEPOWER_ARM;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.LEVEL_1_ASCENT;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.SCORE_IN_BASKET;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;

public class NET_Score_5_PARTNER_PRELOAD extends Routes {
    public NET_Score_5_PARTNER_PRELOAD(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public static final double SLOW_VELOCITY_OVERRIDE = 10;
    public static final double SLOW_ACCELERATION_OVERRIDE = 15;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    public static final double NORMAL_VELOCITY_OVERRIDE = 27;
    public static final double NORMAL_ACCELERATION_OVERRIDE = 27;
    public static final double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(180);

    public static final double FAST_VELOCITY_OVERRIDE = 35;
    public static final double FAST_ACCELERATION_OVERRIDE = 35;
    public static final double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    // Shared constraints for all routes
    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint normalVelocity;
    public static AccelConstraint normalAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;

    public void buildRoute()
    {
        setupConstraints();
        moveFromStartToBasket();
        moveToNeutralSample1();
        moveFromNeutralSample1ToBasket();
        moveToNeutralSample2();
        moveFromNeutralSample2ToBasket();
        moveToNeutralSample3();
        moveFromNeutralSample3ToBasket();
        pickupPartnerPreloadFromGround();
        moveFromPartnerPreloadToBasket();
//        goToSubmersibleAndFish();
//        moveFromSubmersibleToBasket();
        travelToAscentZone();
        netBotRoute = netTrajectoryActionBuilder.build();
    }


    private void setupConstraints() {
        slowVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SLOW_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)
        ));
        slowAcceleration = new ProfileAccelConstraint(-SLOW_ACCELERATION_OVERRIDE, SLOW_ACCELERATION_OVERRIDE);

        normalVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(NORMAL_VELOCITY_OVERRIDE),
                new AngularVelConstraint(NORMAL_ANGULAR_VELOCITY_OVERRIDE)
        ));
        normalAcceleration = new ProfileAccelConstraint(-NORMAL_ACCELERATION_OVERRIDE, NORMAL_ACCELERATION_OVERRIDE);

        fastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)
        ));
        fastAcceleration = new ProfileAccelConstraint(-FAST_ACCELERATION_OVERRIDE, FAST_ACCELERATION_OVERRIDE);
    }

    private void moveFromStartToBasket() {
        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE)
                .setTangent(ANGLE_160_DEGREES)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, fastVelocity, fastAcceleration)
                .afterDisp(4, robotAdapter.getAction(SCORE_IN_BASKET))
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void moveToNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .afterDisp(5.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .splineToLinearHeading(NET_SPIKE_ONE, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .afterDisp(.5, robotAdapter.getAction(PICKUP_FROM_GROUND)) //1.1 worked
                .splineToSplineHeading(NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(SAMPLE_LENGTH/2,0),0)), ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_PICKUP))
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_TRANSFER));
    }

    void moveFromNeutralSample1ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration)
                //todo SHORT side is different than the others right now. need to test this
                .afterDisp(4, robotAdapter.getAction(SCORE_IN_BASKET))
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void moveToNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(5.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_TWO, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .afterDisp(.5, robotAdapter.getAction(PICKUP_FROM_GROUND))
                .splineToSplineHeading(NET_SPIKE_TWO.plus(new Twist2d(new Vector2d(SAMPLE_LENGTH/2,0),0)), ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_PICKUP))
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_TRANSFER));
    }

    void moveFromNeutralSample2ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration)
                .afterDisp(4, robotAdapter.getAction(SCORE_IN_BASKET))
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void moveToNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .afterDisp(7.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .splineToLinearHeading(NET_SPIKE_THREE.plus(new Twist2d(new Vector2d(0,-.8), 0)), ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .afterDisp(.5, robotAdapter.getAction(PICKUP_FROM_GROUND))
                .splineToSplineHeading(NET_SPIKE_THREE.plus(new Twist2d(new Vector2d(SAMPLE_LENGTH/2,-.8),0)), ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_PICKUP))
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_TRANSFER));
    }

    void moveFromNeutralSample3ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration)
                .afterDisp(4, robotAdapter.getAction(SCORE_IN_BASKET))
                .splineToLinearHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void goToSubmersibleAndFish() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(5.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION, fastVelocity, fastAcceleration)
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .afterDisp(9, robotAdapter.getAction(PICKUP_FROM_GROUND))
                .splineToSplineHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_PICKUP))
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_TRANSFER));
    }

    private void moveFromSubmersibleToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(17, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, fastVelocity, fastAcceleration)
                .afterDisp(4, robotAdapter.getAction(SCORE_IN_BASKET))
                .setTangent(ANGLE_225_DEGREES)
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void travelToAscentZone() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(5.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(6, robotAdapter.getAction(LEVEL_1_ASCENT))
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION, fastVelocity, fastAcceleration)
                .splineToLinearHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION, slowVelocity, slowAcceleration)
                .stopAndAdd(robotAdapter.getAction(DEPOWER_ARM));
    }


    public void pickupPartnerPreloadFromGround() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .afterDisp(7.0, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(20, robotAdapter.getAction(PICKUP_FROM_GROUND))
                .splineToLinearHeading(PARTNER_PRELOAD_LEFT_BEHIND, ANGLE_TOWARD_OBSERVATION, fastVelocity, fastAcceleration)
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_PICKUP))
                .stopAndAdd(robotAdapter.getAction(CONDITIONAL_TRANSFER));
    }

    private void moveFromPartnerPreloadToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(6, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .afterDisp(3, robotAdapter.getAction(FLIP_UP_AND_RETRACT))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, fastVelocity, fastAcceleration)
                .afterDisp(4, robotAdapter.getAction(SCORE_IN_BASKET))
                .setTangent(ANGLE_225_DEGREES)
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

}