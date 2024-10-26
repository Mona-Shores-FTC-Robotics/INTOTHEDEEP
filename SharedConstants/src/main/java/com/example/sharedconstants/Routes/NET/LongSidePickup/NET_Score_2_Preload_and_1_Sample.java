package com.example.sharedconstants.Routes.NET.LongSidePickup;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_WAYPOINT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DEPOSIT_SAMPLE;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload;

public class NET_Score_2_Preload_and_1_Sample extends NET_Score_1_Specimen_Preload {
    public NET_Score_2_Preload_and_1_Sample(RobotAdapter robotAdapter) {
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
                .strafeToLinearHeading(PoseToVector(NET_BASKET), ANGLE_45_DEGREES)
                .stopAndAdd(robotAdapter.getAction(DEPOSIT_SAMPLE));
    }

    public void pickupNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(Math.toRadians(230))
                .splineToSplineHeading(NET_SPIKE_ONE_WAYPOINT, ANGLE_TOWARD_BLUE)
                .strafeTo(PoseToVector(NET_SPIKE_ONE))
                .stopAndAdd(robotAdapter.getAction(SAMPLE_INTAKE_ON))
                .strafeTo(PoseToVector(NET_SPIKE_ONE_PICKUP));

    }
}
