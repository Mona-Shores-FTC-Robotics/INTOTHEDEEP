package com.example.sharedconstants.Routes.NET.SpecimenPreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO_APPROACH;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_3_Preload_and_2_Samples extends NET_Score_2_Preload_and_1_Sample {

    public NET_Score_3_Preload_and_2_Samples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute()
    {
        super.buildRoute();
        pickupNeutralSample2();
        scoreSampleInHighBasket();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void pickupNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(6, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_TWO_APPROACH, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .splineToLinearHeading(NET_SPIKE_TWO, ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .waitSeconds(.3);
    }
}
