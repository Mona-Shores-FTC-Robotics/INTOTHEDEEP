package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.FACE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_WAYPOINT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DEPOSIT_SAMPLE;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;

public class NET_Net_Preload_and_One_Sample extends NetPreload {
    public NET_Net_Preload_and_One_Sample(RobotAdapter robotAdapter) {
        super(robotAdapter);
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(Math.toRadians(230))
                .splineToSplineHeading(NET_SPIKE_ONE_WAYPOINT, ANGLE_TOWARD_BLUE)
                .strafeTo(PoseToVector(NET_SPIKE_ONE))
                .stopAndAdd(robotAdapter.getAction(SAMPLE_INTAKE_ON))
                .strafeTo(PoseToVector(NET_SPIKE_ONE_PICKUP))
                .strafeToLinearHeading(PoseToVector(NET_BASKET),FACE_45_DEGREES)
                .stopAndAdd(robotAdapter.getAction(DEPOSIT_SAMPLE));
        netBotRoute = netTrajectoryActionBuilder.build();
    }
}
