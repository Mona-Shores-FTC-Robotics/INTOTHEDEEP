package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class OBS_Retry_Test extends Routes {

    public OBS_Retry_Test(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        retryTest(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void retryTest(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .setTangent(ANGLE_45_DEGREES)
                .splineToLinearHeading(chamberSlot, CHAMBER_SLOT_ONE.heading.toDouble())

                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.SIMPLE_DRIVE));
    }

}
