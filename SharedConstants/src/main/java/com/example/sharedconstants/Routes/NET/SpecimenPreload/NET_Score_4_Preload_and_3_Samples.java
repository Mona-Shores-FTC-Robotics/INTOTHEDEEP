package com.example.sharedconstants.Routes.NET.SpecimenPreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_APPROACH;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_4_Preload_and_3_Samples extends NET_Score_3_Preload_and_2_Samples {
    public NET_Score_4_Preload_and_3_Samples(RobotAdapter robotAdapter) {
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
                .setTangent(ANGLE_TOWARD_BLUE)
                .afterDisp(3, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_THREE_APPROACH, ANGLE_TOWARD_BLUE, normalVelocity, normalAcceleration)
                .splineToLinearHeading(NET_SPIKE_THREE, ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .waitSeconds(.3);
    }
}
