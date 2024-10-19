package com.example.sharedconstants.Routes.NET.ShortSidePickup;

import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_NEUTRAL_SIDE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_SHORT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DEPOSIT_SAMPLE;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Preload;

public class NET_Score_2_Preload_and_1_Sample_Short extends NET_Score_1_Preload {
    public NET_Score_2_Preload_and_1_Sample_Short(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        pickupNeutralSample1();
        depositSample();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    public void depositSample() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .strafeToLinearHeading(PoseToVector(NET_BASKET_NEUTRAL_SIDE), ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(DEPOSIT_SAMPLE));
    }

    private void pickupNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToLinearHeading(NET_SPIKE_ONE_SHORT, ANGLE_TOWARD_BLUE)
                .afterDisp(1, robotAdapter.getAction(SAMPLE_INTAKE_ON));
    }
}
