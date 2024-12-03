package com.example.sharedconstants.Routes.NET.SpecimenPreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.HUMAN_PLAYER_SAMPLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.HUMAN_PLAYER_SAMPLE_STAGING;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples extends NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample {
    public NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pickupHumanPlayerSample2();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    public void pickupHumanPlayerSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_OBSERVATION)
                .splineToSplineHeading(HUMAN_PLAYER_SAMPLE_STAGING, ANGLE_TOWARD_OBSERVATION)
                .strafeTo(PoseToVector(HUMAN_PLAYER_SAMPLE_PICKUP));
    }

}
