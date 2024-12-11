package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE_APPROACH;
import static com.example.sharedconstants.FieldConstants.NEXT_TO_NET_ASCENT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.SAMPLE_LENGTH;
import static com.example.sharedconstants.RobotAdapter.ActionType.DEPOWER_ARM;
import static com.example.sharedconstants.RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE;
import static com.example.sharedconstants.RobotAdapter.ActionType.FLIP_UP_AND_RETRACT;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION;
import static com.example.sharedconstants.RobotAdapter.ActionType.LEVEL_1_ASCENT;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_LOW_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;

public class NET_Score_4_Sample_Preload extends NET_Score_3_Sample_Preload {
    public NET_Score_4_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute()
    {
        super.buildRoute();
        moveToNeutralSample3();
        moveFromNeutralSample3ToBasket();
        scoreSampleInHighBasket();
        travelToAscentZone();
        netBotRoute = netTrajectoryActionBuilder.build();
    }

    private void moveToNeutralSample3() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setTangent(ANGLE_45_DEGREES)
                .afterDisp(7.0, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(0, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(NET_SPIKE_THREE.plus(new Twist2d(new Vector2d(SAMPLE_LENGTH/2,-.8),0)), ANGLE_115_DEGREES, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND))
                .waitSeconds(2.5);
    }

    void moveFromNeutralSample3ToBasket() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .afterDisp(0, robotAdapter.getAction(FLIP_UP_AND_RETRACT))
                .splineToLinearHeading(NET_BASKET_AUTO, ANGLE_225_DEGREES, normalVelocity, normalAcceleration);
    }

    private void travelToAscentZone() {
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .afterDisp(5.5, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME))
                .afterDisp(6, robotAdapter.getAction(LEVEL_1_ASCENT))
                .splineToLinearHeading(NEXT_TO_NET_ASCENT, ANGLE_TOWARD_OBSERVATION, fastVelocity, fastAcceleration)
                .splineToLinearHeading(NET_ASCENT, ANGLE_TOWARD_OBSERVATION, normalVelocity, normalAcceleration)
                .stopAndAdd(robotAdapter.getAction(DEPOWER_ARM));
    }

}
