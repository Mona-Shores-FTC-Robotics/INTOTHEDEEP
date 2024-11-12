package com.example.sharedconstants.Routes.OBS.InakeAndScore;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_BEFORE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_DUMP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP_FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_1;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_2;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_3;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;

public class OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike extends OBS_Score_1_Specimen_Preload {

    public OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.buildRoute();
        intakeFirstTeamSpecimen();
        intakeSecondTeamSpecimen();
        intakeThirdTeamSpecimen();
        pickupSpecimenFromWallAndScore(CHAMBER_SLOT_FOUR);
        pickupSpecimenFromWallAndScore(CHAMBER_SLOT_THREE);
        pickupSpecimenFromWallAndScore(CHAMBER_SLOT_TWO);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickupSpecimenFromWallAndScore(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_BEFORE_PICKUP,ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_ZONE_PICKUP_FACE_TOWARD_BLUE,ANGLE_TOWARD_RED)
                .waitSeconds(.2)
                .splineToLinearHeading(chamberSlot,ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER))
                .setReversed(true);
    }

    public void intakeFirstTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_1, ANGLE_TOWARD_BLUE)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .waitSeconds(1);
   }

    public void intakeSecondTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_2, ANGLE_TOWARD_BLUE)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .waitSeconds(1);
    }

    public void intakeThirdTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_3, ANGLE_TOWARD_OBSERVATION)
                .waitSeconds(1);
    }
}