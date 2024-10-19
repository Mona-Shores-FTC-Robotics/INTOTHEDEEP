package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.SECURE_PRELOAD_SPECIMEN;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class ObservationPreload extends Routes {
    public ObservationPreload(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        scoreObservationPreload(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreObservationPreload(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(OBS_START_POSE)
                .splineToLinearHeading(chamberSlot, CHAMBER_SLOT_ONE.heading.toDouble())
                .afterDisp(0, robotAdapter.getAction(SECURE_PRELOAD_SPECIMEN))
                .afterDisp(.1, robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction((HANG_SPECIMEN_ON_HIGH_CHAMBER)))
                .strafeTo(PoseToVector(CHAMBER_SLOT_ONE).minus(new Vector2d(0, 3)))
                .afterDisp(0, robotAdapter.getAction(HOME));
    }


}
