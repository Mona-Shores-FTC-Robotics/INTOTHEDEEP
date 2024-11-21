package com.example.sharedconstants.Routes.NET.SamplePreload;

import static com.example.sharedconstants.FieldConstants.ANGLE_160_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN_ROTATED;
import static com.example.sharedconstants.FieldConstants.NET_BASKET;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_PRE_SCORE;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.MOVE_PRELOAD_SPECIMEN_TO_CW_HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.PREPARE_TO_SCORE_IN_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SCORE_IN_HIGH_BASKET;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
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

    private static final double PRELOAD_SLOW_VELOCITY_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ACCELERATION_OVERRIDE = 18;
    private static final double PRELOAD_SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);


    public void buildRoute() {
        scoreNetSamplePreload();
        netBotRoute= netTrajectoryActionBuilder.build();
    }

    public void scoreNetSamplePreload(){
        VelConstraint preloadVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(PRELOAD_VELOCITY_OVERRIDE),
                new AngularVelConstraint(PRELOAD_ANGULAR_VELOCITY_OVERRIDE)
        ));
        AccelConstraint preloadAcceleration = new ProfileAccelConstraint(- PRELOAD_ACCELERATION_OVERRIDE, PRELOAD_ACCELERATION_OVERRIDE);

        netTrajectoryActionBuilder = robotAdapter.getActionBuilder(NET_START_POSE)
                .setTangent(ANGLE_160_DEGREES)
                .splineToSplineHeading(NET_BASKET_PRE_SCORE, ANGLE_TOWARD_NET, preloadVelocity, preloadAcceleration)
                .stopAndAdd(robotAdapter.getAction(PREPARE_TO_SCORE_IN_HIGH_BASKET))
                .stopAndAdd(robotAdapter.getAction(SCORE_IN_HIGH_BASKET));
    }
}
