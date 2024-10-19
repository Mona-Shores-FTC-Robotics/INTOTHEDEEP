package com.example.sharedconstants.Routes.NET.LongSidePickup;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples extends NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample {
    public NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pickupHumanPlayerSample();
        depositSampleWall();
        netBotRoute = netTrajectoryActionBuilder.build();
    }
}
