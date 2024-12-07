
package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_APPROACH;
import static com.example.sharedconstants.FieldConstants.SAMPLE_LENGTH;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_LOW_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
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
                .afterDisp(5.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(SAMPLE_LENGTH/2,0),0)), ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(2.25);
    }

    void moveFromNeutralSample1ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .afterDisp(0, robotAdapter.getAction(FLIP_UP_AND_RETRACT))
                .splineToLinearHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

}
