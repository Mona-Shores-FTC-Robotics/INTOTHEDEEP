package com.example.sharedconstants.Routes.NET.LongSidePickup;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_3_Preload_and_2_Samples extends NET_Score_2_Preload_and_1_Sample {

    public NET_Score_3_Preload_and_2_Samples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        pickupNeutralSample2();
        depositSample();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void pickupNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(NET_SPIKE_TWO, ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(SAMPLE_INTAKE_ON))
                .strafeTo(PoseToVector(NET_SPIKE_TWO_PICKUP));
    }
}
