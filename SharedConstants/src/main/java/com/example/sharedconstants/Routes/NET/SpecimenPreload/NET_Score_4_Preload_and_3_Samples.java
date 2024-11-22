package com.example.sharedconstants.Routes.NET.SpecimenPreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_APPROACH;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_NET_ASCENT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.example.sharedconstants.RobotAdapter;

public class NET_Score_4_Preload_and_3_Samples extends NET_Score_3_Preload_and_2_Samples {
    public NET_Score_4_Preload_and_3_Samples(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        super.buildRoute();
        moveToNeutralSample3();
        pickupNeutralSample3();
        moveFromNeutralSample3ToBasket();
        scoreSampleInHighBasket();
        travelToAscentZone();

        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void moveToNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(3, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_THREE_APPROACH, ANGLE_115_DEGREES, normalVelocity, normalAcceleration);
    }

    private void pickupNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_BLUE)
                .splineToLinearHeading(NET_SPIKE_THREE, ANGLE_TOWARD_BLUE, slowVelocity, slowAcceleration);
    }

    void moveFromNeutralSample3ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(NET_BASKET, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void travelToAscentZone() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration)
                .splineToLinearHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration);
        //TODO add arm movements to achieve level 1 ascend

    }
}
