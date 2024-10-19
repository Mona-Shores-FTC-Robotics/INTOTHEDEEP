package com.example.sharedconstants.Routes.OBS.Push;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;
import static com.example.sharedconstants.FieldConstants.FACE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.OBS_WAYPOINT;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL;

import com.example.sharedconstants.RobotAdapter;

public class OBS___Score___Push___Four_NeutralScoreFour_Specimens extends OBS_Score_Push_Three_Neutral_Score_Four_Specimens {

    public OBS___Score___Push___Four_NeutralScoreFour_Specimens(RobotAdapter roadRunnerDrive) {
        super(roadRunnerDrive);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(FACE_315_DEGREES)
                .splineToLinearHeading(OBS_WAYPOINT, ANGLE_TOWARD_RED)
                .afterDisp(.1, robotAdapter.getAction(HOME))
                .splineTo(PoseToVector(OBS_ZONE_PICKUP), ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(PICKUP_SPECIMEN_OFF_WALL))
                .setReversed(true)
                .splineToLinearHeading(CHAMBER_SLOT_FIVE, ANGLE_TOWARD_BLUE)
                .afterDisp(.1, robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }
}
