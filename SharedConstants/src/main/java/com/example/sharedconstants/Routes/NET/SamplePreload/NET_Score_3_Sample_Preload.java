package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO_APPROACH;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_3_Sample_Preload extends NET_Score_2_Sample_Preload {
    public NET_Score_3_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        moveToNeutralSample2();
        pickupNeutralSample2();
        moveFromNeutralSample2ToBasket();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void moveToNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(3, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_TWO_APPROACH, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration);
    }

    private void pickupNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToSplineHeading(NET_SPIKE_TWO, ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .waitSeconds(1.5);

    }

    void moveFromNeutralSample2ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .afterDisp(0, robotAdapter.getAction(FLIP_UP_AND_RETRACT))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration)
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }
}
