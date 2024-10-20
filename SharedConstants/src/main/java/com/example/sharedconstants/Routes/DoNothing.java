package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_DO_NOTHING;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_DO_NOTHING;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;

public class DoNothing extends Routes {

    public DoNothing(RobotAdapter robotAdapter) {
        super(robotAdapter);

        netBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .strafeTo(PoseToVector(NET_DO_NOTHING))
                .build();

        observationBotRoute = robotAdapter.getActionBuilder(OBS_START_POSE)
                .strafeTo(PoseToVector(OBS_DO_NOTHING))
                .build();
    }
}

