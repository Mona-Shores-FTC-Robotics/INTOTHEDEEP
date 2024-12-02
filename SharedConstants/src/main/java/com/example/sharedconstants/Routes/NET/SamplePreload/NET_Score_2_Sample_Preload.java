
package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_APPROACH;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_LOW_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.WAIT_FOR_PICKUP_OR_TIMEOUT;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_2_Sample_Preload extends NET_Score_1_Sample_Preload {

    public NET_Score_2_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        moveToNeutralSample1();
        moveFromNeutralSample1ToBasket();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void moveToNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(4, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_ONE, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .stopAndAdd(robotAdapter.getAction(WAIT_FOR_PICKUP_OR_TIMEOUT));
    }

    void moveFromNeutralSample1ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
//                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToSplineHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, slowVelocity, slowAcceleration);
    }

}
