package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class OBS_Intake_Automatic_Sample_Handling extends Routes {

    public OBS_Intake_Automatic_Sample_Handling(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE)
                .stopAndAdd(robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineTo(PoseToVector(OBS_START_POSE).plus(new Vector2d(0, 24)), ANGLE_TOWARD_BLUE, slowVelocity)
                .setReversed(true)
                .splineTo(PoseToVector(OBS_START_POSE), ANGLE_TOWARD_RED, slowVelocity);
    }

}