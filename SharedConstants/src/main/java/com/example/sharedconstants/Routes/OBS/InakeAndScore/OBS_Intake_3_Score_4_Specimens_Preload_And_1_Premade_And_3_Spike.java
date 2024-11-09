package com.example.sharedconstants.Routes.OBS.InakeAndScore;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_INTAKE_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_INTAKE_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_INTAKE_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_DUMP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP_FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_1;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_2;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_3;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.MOVE_PRELOAD_SPECIMEN_TO_CW_HOME;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike;

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
//        intakeThirdTeamSpecimen();
//        pickupSpecimenFromWall();
//        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }



    public void pickupSpecimenFromWallAndScore(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_ZONE_PICKUP_FACE_TOWARD_BLUE,ANGLE_TOWARD_RED)
                .waitSeconds(.2)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.PICKUP_SPECIMEN_OFF_WALL))
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

//                .splineToSplineHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_OBSERVATION)
//                .splineToConstantHeading(PoseToVector(OBS_SPIKE_ONE), ANGLE_TOWARD_RED)
//                .splineToConstantHeading(PoseToVector(OBS_INTAKE_SPIKE_ONE), ANGLE_TOWARD_BLUE);
    }

    public void intakeSecondTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_2, ANGLE_TOWARD_BLUE)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .waitSeconds(1);
//                .splineToSplineHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_OBSERVATION)
//                .splineToConstantHeading(PoseToVector(OBS_SPIKE_THREE), ANGLE_TOWARD_RED)
//                .splineToConstantHeading(PoseToVector(OBS_INTAKE_SPIKE_THREE), ANGLE_TOWARD_RED);
    }


    public void intakeThirdTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_3, ANGLE_TOWARD_OBSERVATION)
                .waitSeconds(1);
//                .splineToSplineHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_OBSERVATION)
//                .splineToConstantHeading(PoseToVector(OBS_SPIKE_THREE), ANGLE_TOWARD_RED)
//                .splineToConstantHeading(PoseToVector(OBS_INTAKE_SPIKE_THREE), ANGLE_TOWARD_RED);
    }
}



