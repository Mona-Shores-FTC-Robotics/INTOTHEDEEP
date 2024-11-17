package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.HUMAN_PLAYER_SAMPLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.HUMAN_PLAYER_SAMPLE_STAGING;
import static com.example.sharedconstants.FieldConstants.NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_NEUTRAL_SIDE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SCORE_IN_HIGH_BASKET;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;

@AutoRoute
public class NET_Score5_SamplePreload extends Routes {
    public NET_Score5_SamplePreload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute() {
        scorePreloadSampleInBasket();
        pickupNeutralSample1();
        depositSample();
        pickupNeutralSample2();
        depositSample();
        pickupNeutralSample3();
        depositSample();
        pickupHumanPlayerSample1();
        depositSampleWall();
        travelToAscentZone();
        netBotRoute= netTrajectoryActionBuilder.build();
    }

    private void travelToAscentZone() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION)
                .splineToLinearHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION);
    }

    public void scorePreloadSampleInBasket() {
        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE)
                .setReversed(true)
                .afterDisp(10, robotAdapter.getAction((PREPARE_TO_SCORE_IN_HIGH_BASKET)))
                .splineToLinearHeading(NET_BASKET, ANGLE_TOWARD_NET)
                .stopAndAdd(robotAdapter.getAction(SCORE_IN_HIGH_BASKET));
    }


    public void depositSample() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(4, robotAdapter.getAction((PREPARE_TO_SCORE_IN_HIGH_BASKET)))
                .strafeToLinearHeading(PoseToVector(NET_BASKET_NEUTRAL_SIDE), ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(SCORE_IN_HIGH_BASKET));
    }

    private void pickupNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(1, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_ONE , ANGLE_TOWARD_BLUE);
    }

    private void pickupNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(1, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToConstantHeading(PoseToVector(NET_SPIKE_TWO), ANGLE_TOWARD_BLUE);
    }

    private void pickupNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(1, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToConstantHeading(PoseToVector(NET_SPIKE_THREE), ANGLE_TOWARD_BLUE);
    }

public void pickupHumanPlayerSample1() {
    netTrajectoryActionBuilder = netTrajectoryActionBuilder
            .setTangent(Math.toRadians(338))
            .splineToSplineHeading(HUMAN_PLAYER_SAMPLE_STAGING, ANGLE_TOWARD_OBSERVATION)
            .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
            .strafeTo(PoseToVector(HUMAN_PLAYER_SAMPLE_PICKUP), slowVelocity);
}

    public void depositSampleWall()
    {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(2, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .strafeToConstantHeading(PoseToVector(NET_BASKET));
    }
}
