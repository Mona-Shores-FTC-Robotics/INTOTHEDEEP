package com.example.sharedconstants.Routes.OBS.InakeAndScore;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_DUMP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP_FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_1;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_2;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_3;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;

public class OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike_Not_At_1_Time extends OBS_Score_1_Specimen_Preload {

    public OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike_Not_At_1_Time(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        super.buildRoute();
        intakeFirstTeamSpecimen();
        pickupSpecimenFromWallAndScore();
        intakeSecondTeamSpecimen();
        pickupSpecimenFromWallAndScore();
        intakeThirdTeamSpecimen();
        pickupSpecimenFromWallAndScore();
//        intakeThirdTeamSpecimen();
//        pickupSpecimenFromWall();
//        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }



    public void pickupSpecimenFromWallAndScore() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_RED)
                .splineToLinearHeading(OBS_ZONE_PICKUP_FACE_TOWARD_BLUE,ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL))
                .splineToLinearHeading(CHAMBER_SLOT_FOUR,ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER));

    }

    public void intakeFirstTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_1, ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.INTAKE_SAMPLE_FROM_GROUND))
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE));
    }

    public void intakeSecondTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_2, ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.INTAKE_SAMPLE_FROM_GROUND))
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE));

    }


    public void intakeThirdTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_DUMP, ANGLE_TOWARD_OBSERVATION)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.INTAKE_SAMPLE_FROM_GROUND));

    }
}



