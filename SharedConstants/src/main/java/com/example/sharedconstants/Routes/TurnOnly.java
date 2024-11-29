package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.example.sharedconstants.RobotAdapter;

@AutoRoute  // Add this annotation
public class TurnOnly extends Routes {

    public TurnOnly(RobotAdapter robotAdapter) {
        super(robotAdapter);

        netBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .turnTo(ANGLE_TOWARD_BLUE)
                .build();

        observationBotRoute = robotAdapter.getActionBuilder(OBS_START_POSE)
                .turnTo(ANGLE_TOWARD_BLUE)
                .build();
    }
}

