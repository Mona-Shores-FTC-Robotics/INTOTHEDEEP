package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_160_DEGREES;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_ALIGNMENT_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_AUTO;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SCORE_IN_HIGH_BASKET;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.AutoRoute;
import com.example.sharedconstants.Routes.Routes;

import java.util.Arrays;


@AutoRoute
public class NET_Score_1_Sample_Preload extends Routes {

    public NET_Score_1_Sample_Preload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    private static final double PRELOAD_VELOCITY_OVERRIDE = 24;
    private static final double PRELOAD_ACCELERATION_OVERRIDE = 24;
    private static final double PRELOAD_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    public void buildRoute() {
        moveFromStartToBasket();
        scoreSampleInHighBasket();
        netBotRoute= netTrajectoryActionBuilder.build();
    }

    private void moveFromStartToBasket() {
        VelConstraint preloadVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(PRELOAD_VELOCITY_OVERRIDE),
                new AngularVelConstraint(PRELOAD_ANGULAR_VELOCITY_OVERRIDE)
        ));
        AccelConstraint preloadAcceleration = new ProfileAccelConstraint(- PRELOAD_ACCELERATION_OVERRIDE, PRELOAD_ACCELERATION_OVERRIDE);

        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE)
                .setTangent(ANGLE_160_DEGREES)
                .afterDisp(10, robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .splineToLinearHeading(NET_BASKET_ALIGNMENT_AUTO, Math.toRadians(235), preloadVelocity, preloadAcceleration)
                .splineToSplineHeading(NET_BASKET_AUTO, Math.toRadians(235), preloadVelocity, preloadAcceleration);
    }

    public void scoreSampleInHighBasket(){
        netTrajectoryActionBuilder = netTrajectoryActionBuilder
                .stopAndAdd(robotAdapter.getAction(SCORE_IN_HIGH_BASKET))
                .waitSeconds(.5);
    }
}
