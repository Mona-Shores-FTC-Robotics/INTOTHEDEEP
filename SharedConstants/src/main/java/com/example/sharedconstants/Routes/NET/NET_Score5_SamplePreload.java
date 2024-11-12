package com.example.sharedconstants.Routes.NET;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.HUMAN_PLAYER_SAMPLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.HUMAN_PLAYER_SAMPLE_STAGING;
import static com.example.sharedconstants.FieldConstants.NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_NEUTRAL_SIDE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_WALL;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_SHORT;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_SHORT;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO_SHORT;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE_WITH_SAMPLE_PRELOAD;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DUMP_SAMPLE_IN_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_INTAKE_ON;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;

@AutoRoute
public class NET_Score5_SamplePreload extends Routes {
    public NET_Score5_SamplePreload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute() {
        scorePreloadSampleInBasket();
        pickupNeutralSample1();
        depositSample();
        pickupNeutralSample2();
        depositSample();
        pickupNeutralSample3();
        depositSample();
        pickupHumanPlayerSample1();
        depositSampleWall();
        travelToAscentZone();
        netBotRoute= netTrajectoryActionBuilder.build();
    }

    private void travelToAscentZone() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION)
                .splineToLinearHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION);
    }

    public void scorePreloadSampleInBasket() {
        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE_WITH_SAMPLE_PRELOAD)
                .strafeTo(PoseToVector(NET_BASKET_WALL))
                .afterDisp(2, robotAdapter.getAction(SAMPLE_LIFT_TO_HIGH_BASKET))
                .stopAndAdd(robotAdapter.getAction((DUMP_SAMPLE_IN_BASKET)));
    }


    public void depositSample() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .strafeToLinearHeading(PoseToVector(NET_BASKET_NEUTRAL_SIDE), ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(DUMP_SAMPLE_IN_BASKET));
    }

    private void pickupNeutralSample1() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToLinearHeading(NET_SPIKE_ONE_SHORT, ANGLE_TOWARD_BLUE)
                .afterDisp(1, robotAdapter.getAction(SAMPLE_INTAKE_ON));
    }

    private void pickupNeutralSample2() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(NET_SPIKE_TWO_SHORT), ANGLE_TOWARD_BLUE)
                .afterDisp(.5, robotAdapter.getAction(SAMPLE_INTAKE_ON));
    }

    private void pickupNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(NET_SPIKE_THREE_SHORT), ANGLE_TOWARD_BLUE)
                .afterDisp(.5, robotAdapter.getAction(SAMPLE_INTAKE_ON));
    }




public void pickupHumanPlayerSample1() {
    netTrajectoryActionBuilder = netTrajectoryActionBuilder
            .setTangent(Math.toRadians(338))
            .splineToSplineHeading(HUMAN_PLAYER_SAMPLE_STAGING, ANGLE_TOWARD_OBSERVATION)
            .stopAndAdd(robotAdapter.getAction(SAMPLE_INTAKE_ON))
            .strafeTo(PoseToVector(HUMAN_PLAYER_SAMPLE_PICKUP));
}

public void depositSampleWall()
{
    netTrajectoryActionBuilder = netTrajectoryActionBuilder
            .strafeToConstantHeading(PoseToVector(NET_BASKET_WALL))
            .stopAndAdd(robotAdapter.getAction(DUMP_SAMPLE_IN_BASKET));

}
}
