package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.FACE_TOWARD_BACKSTAGE;

import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_WAYPOINT;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HOME;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;

public class OBS_Score_Preload_Push_One_Neutral_Score_One_Premade_Specimen extends ObservationPreload {

    public OBS_Score_Preload_Push_One_Neutral_Score_One_Premade_Specimen(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute(){
        super.scoreObservationPreload(CHAMBER_SLOT_ONE);
        pushFirstNeutralSpecimen();
        pickupSpecimenFromWall();
        scoreSpecimenOnChamber(CHAMBER_SLOT_TWO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scoreSpecimenOnChamber(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToSplineHeading(chamberSlot, ANGLE_TOWARD_BLUE)
                .afterDisp(.4, robotAdapter.getAction(RobotAdapter.ActionType.LIFT_TO_HIGH_CHAMBER))
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER))
                .strafeTo(PoseToVector(CHAMBER_SLOT_TWO).minus(new Vector2d(0,3)))
                .afterDisp(0, robotAdapter.getAction(HOME));
    }

    public void pickupSpecimenFromWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(OBS_ZONE_PICKUP, ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL));
    }

    public void pushFirstNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(Math.toRadians(-10))
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_ONE), FACE_TOWARD_BACKSTAGE)
                .splineToLinearHeading(OBS_WAYPOINT, ANGLE_TOWARD_RED);
    }
}

