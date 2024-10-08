package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.FACE_135_DEGREES;
import static com.example.sharedconstants.FieldConstants.FACE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.NET_CHAMBER;
import static com.example.sharedconstants.FieldConstants.NET_CHAMBER_PRELOAD;
import static com.example.sharedconstants.FieldConstants.NET_POS_AUDIENCE_TJ;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_BEHIND;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_ONE_VEC;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TEST;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TEST_TURN;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.NET_SPIKE_TWO_VEC;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_PRELOAD;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_ZONE_RED_PICKUP;
import static com.example.sharedconstants.FieldConstants.NET_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBSERVATION_START_POSE;
import static com.example.sharedconstants.FieldConstants.OBS_CHAMBER_TWO;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ;
import static com.example.sharedconstants.FieldConstants.WALL_ALIGN_POS_AUDIENCE_TJ;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.RobotAdapter;

public class NET_Preload_and_One_Sample extends Routes {

    public NET_Preload_and_One_Sample(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
    }

    public void BuildRoutes() {
        RouteBuilder routeBuilder = new RouteBuilder();

        TrajectoryActionBuilder builder = robotAdapter.getActionBuilder(NET_START_POSE);

        //We can't mix and match the routeBuilder methods with actual movements (e.g., splineToLinearHeading) because this main actionbuilder does not know where the robot is.
        //not sure how to fix this, tried a lot of stuff

        netBotRoute = builder
                .stopAndAdd(routeBuilder.ScorePreloadSpecimen(NET_START_POSE, NET_CHAMBER))
                .stopAndAdd(routeBuilder.PickupSample(NET_CHAMBER,NET_SPIKE_TEST))
//                .stopAndAdd(routeBuilder.GotoWall(NET_SPIKE_TEST, WALL_ALIGN_POS_AUDIENCE_TJ))
                .stopAndAdd(routeBuilder.ScoreSample(NET_SPIKE_TEST, NET_POS_AUDIENCE_TJ))
                .stopAndAdd(routeBuilder.NullDriveAction(NET_POS_AUDIENCE_TJ))
                .build();
    }
}
