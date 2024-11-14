package com.example.sharedconstants.Routes.OBS.InakeAndScore;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_BEFORE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP_FACE_TOWARD_BLUE;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;

public class OBS_Score2_With_Retry extends OBS_Score_1_Specimen_Preload {

    public OBS_Score2_With_Retry(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.buildRoute();
        pickupSpecimenFromWallAndScore(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickupSpecimenFromWallAndScore(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(6, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_ZONE_BEFORE_PICKUP, ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_ZONE_PICKUP_FACE_TOWARD_BLUE, ANGLE_TOWARD_RED, normalVelocity, normalAcceleration)
                .waitSeconds(.5) //see if we can pickup before trying again?
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.SPECIMEN_PICKUP_RETRY))
                .splineToLinearHeading(chamberSlot, ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }
}