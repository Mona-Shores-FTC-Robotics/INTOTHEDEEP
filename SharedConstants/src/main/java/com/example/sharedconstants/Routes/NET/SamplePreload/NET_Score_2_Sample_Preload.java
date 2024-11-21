package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_PRE_SCORE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_APPROACH;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SCORE_IN_HIGH_BASKET;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_1_Specimen_Preload;

public class NET_Score_2_Sample_Preload extends NET_Score_1_Sample_Preload {
    public NET_Score_2_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        pickupNeutralSample1();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void pickupNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(5, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToSplineHeading(NET_SPIKE_ONE_APPROACH, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .splineToConstantHeading(PoseToVector(NET_SPIKE_ONE), ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .waitSeconds(.3);
    }

    public void scoreSampleInHighBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(NET_BASKET_PRE_SCORE, ANGLE_225_DEGREES)
                .stopAndAdd(robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .setReversed(true)
                .splineToLinearHeading(NET_BASKET, ANGLE_225_DEGREES, slowVelocity, slowAcceleration)
                .stopAndAdd(robotAdapter.getAction(SCORE_IN_HIGH_BASKET));
    }


}
