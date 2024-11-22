
package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_APPROACH;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.example.sharedconstants.RobotAdapter;

public class NET_Score_2_Sample_Preload extends NET_Score_1_Sample_Preload {
    public NET_Score_2_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        moveToNeutralSample1();
        pickupNeutralSample1();
        moveFromNeutralSample1ToBasket();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void moveToNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(3, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_ONE_APPROACH, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration);
    }

    private void pickupNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToSplineHeading(NET_SPIKE_ONE, ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration);
    }

    void moveFromNeutralSample1ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(10, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT, ANGLE_225_DEGREES)

                .splineToSplineHeading(NET_BASKET, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

}