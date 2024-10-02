package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.NET_CHAMBER;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_PRELOAD_VEC;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_AUDIENCE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BACKSTAGE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_START_POSE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_VEC;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_TWO_VEC;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE_VEC;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public class BasicRoute extends Routes {

    public BasicRoute(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void BuildRoutes() {
        /** NET ROUTE **/
        netBotRoute = robotAdapter.getActionBuilder(NET_START_POSE)
                .splineToLinearHeading(NET_CHAMBER, FACE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .setReversed(true)
                .splineToConstantHeading(NET_SPIKE_ONE_VEC, FACE_TOWARD_AUDIENCE, slowVelocity, slowAcceleration)
                .build();

        observationBotRoute = robotAdapter.getActionBuilder(OBSERVATION_START_POSE)
                .splineToConstantHeading(OBS_CHAMBER_PRELOAD_VEC, FACE_TOWARD_BLUE, slowVelocity, slowAcceleration)
                .setReversed(true)
                .splineToConstantHeading(OBS_SPIKE_ONE_VEC, FACE_TOWARD_BACKSTAGE, slowVelocity, slowAcceleration)
                .build();

    }
}

