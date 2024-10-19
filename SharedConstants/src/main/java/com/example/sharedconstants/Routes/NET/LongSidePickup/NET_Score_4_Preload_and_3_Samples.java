package com.example.sharedconstants.Routes.NET.LongSidePickup;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_4_Preload_and_3_Samples extends NET_Score_3_Preload_and_2_Samples {
    public NET_Score_4_Preload_and_3_Samples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pickupNeutralSample3();
        depositSample();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void pickupNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_BLUE)
                .splineToSplineHeading(NET_SPIKE_THREE, ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(SAMPLE_INTAKE_ON))
                .strafeTo(PoseToVector(NET_SPIKE_THREE_PICKUP));
    }
}
