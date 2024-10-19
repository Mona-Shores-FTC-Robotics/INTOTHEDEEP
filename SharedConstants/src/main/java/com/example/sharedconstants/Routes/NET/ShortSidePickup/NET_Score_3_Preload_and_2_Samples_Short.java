package com.example.sharedconstants.Routes.NET.ShortSidePickup;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO_SHORT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_3_Preload_and_2_Samples_Short extends NET_Score_2_Preload_and_1_Sample_Short {

    public NET_Score_3_Preload_and_2_Samples_Short(RobotAdapter robotAdapter) {
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
                .splineToConstantHeading(PoseToVector(NET_SPIKE_TWO_SHORT), ANGLE_TOWARD_BLUE)
                .afterDisp(.5, robotAdapter.getAction(SAMPLE_INTAKE_ON));
    }
}
