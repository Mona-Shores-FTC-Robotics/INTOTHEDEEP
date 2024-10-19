package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.FACE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_WAYPOINT;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;
import static com.example.sharedconstants.RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_Preload_and_One_Premade_Specimen extends ObservationPreload {

    public OBS_Score_Preload_and_One_Premade_Specimen(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.scoreObservationPreload(CHAMBER_SLOT_ONE);
        pickupSpecimenFromWall();
        scoreOnHighChamber(CHAMBER_SLOT_TWO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickupSpecimenFromWall(){
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(FACE_315_DEGREES)
                .splineToLinearHeading(OBS_WAYPOINT, ANGLE_TOWARD_RED)
                .afterDisp(.1, robotAdapter.getAction(HOME))
                .splineTo(PoseToVector(OBS_ZONE_PICKUP), ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(PICKUP_SPECIMEN_OFF_WALL));
    }

    public void scoreOnHighChamber(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(chamberSlot, ANGLE_TOWARD_BLUE)
                .afterDisp(.1, robotAdapter.getAction(LIFT_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }
}