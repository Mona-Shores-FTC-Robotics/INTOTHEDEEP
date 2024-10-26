package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.example.sharedconstants.RobotAdapter;

@AutoRoute  // Add this annotation
public class MoveOnly extends Routes {

    public MoveOnly(RobotAdapter robotAdapter) {
        super(robotAdapter);

        netBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .strafeTo(PoseToVector(NET_BASKET))
                .build();

        observationBotRoute = robotAdapter.getActionBuilder(OBS_START_POSE)
                .strafeTo(PoseToVector(OBS_ZONE_PICKUP))
                .build();
    }
}

