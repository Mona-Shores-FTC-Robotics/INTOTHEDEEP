package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.FACE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DEPOSIT_SAMPLE;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;

public class NET_Net_Preload_and_Three_Samples extends NET_Net_Preload_and_Two_Samples {
    public NET_Net_Preload_and_Three_Samples(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_BLUE)
                .splineToSplineHeading(NET_SPIKE_THREE, ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(SAMPLE_INTAKE_ON))
                .strafeTo(PoseToVector(NET_SPIKE_THREE_PICKUP))
                .strafeToLinearHeading(PoseToVector(NET_BASKET),FACE_45_DEGREES)
                .stopAndAdd(robotAdapter.getAction(DEPOSIT_SAMPLE));

        netBotRoute = netTrajectoryActionBuilder.build();
    }
}
