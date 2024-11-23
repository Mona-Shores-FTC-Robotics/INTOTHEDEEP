package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_APPROACH;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_NET_ASCENT;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_4_Sample_Preload extends NET_Score_3_Sample_Preload {
    public NET_Score_4_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        moveToNeutralSample3();
        pickupNeutralSample3();
        moveFromNeutralSample3ToBasket();
        scoreSampleInHighBasket();
        travelToAscentZone();

        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void moveToNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(3, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_THREE_APPROACH, ANGLE_115_DEGREES, normalVelocity, normalAcceleration);
    }

    private void pickupNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_BLUE)
                .splineToLinearHeading(NET_SPIKE_THREE, ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .waitSeconds(1.5);

    }

    void moveFromNeutralSample3ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .afterDisp(0, robotAdapter.getAction(FLIP_UP_AND_RETRACT))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration)
                .splineToLinearHeading(NET_BASKET, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void travelToAscentZone() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration)
                .splineToLinearHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration);
                //TODO add arm movements to achieve level 1 ascend

    }

}
