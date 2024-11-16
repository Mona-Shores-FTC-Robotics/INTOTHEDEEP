package com.example.sharedconstants.Routes.NET.ShortSidePickup;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_SHORT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_4_Preload_and_3_Samples_Short extends NET_Score_3_Preload_and_2_Samples_Short {
    public NET_Score_4_Preload_and_3_Samples_Short(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pickupNeutralSample3();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void pickupNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(.5, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToConstantHeading(PoseToVector(NET_SPIKE_THREE_SHORT), ANGLE_TOWARD_BLUE);
    }
}
