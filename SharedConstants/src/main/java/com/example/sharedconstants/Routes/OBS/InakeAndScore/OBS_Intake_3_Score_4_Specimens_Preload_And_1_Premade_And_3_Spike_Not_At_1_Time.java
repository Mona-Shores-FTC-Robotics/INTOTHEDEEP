package com.example.sharedconstants.Routes.OBS.InakeAndScore;

import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_DUMP;
import static com.example.sharedconstants.FieldConstants.OBS_ZONE_PICKUP_FACE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_1;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_INTAKE_2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
//        pickupSpecimenFromWallAndScore();
//        intakeSecondTeamSpecimen();
//        pickupSpecimenFromWallAndScore();
//        intakeThirdTeamSpecimen();
//        pickupSpecimenFromWallAndScore();
//        intakeThirdTeamSpecimen();
//        pickupSpecimenFromWall();
//        scoreOnHighChamber(CHAMBER_SLOT_FOUR);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }



    public void pickupSpecimenFromWallAndScore() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToLinearHeading(OBS_ZONE_PICKUP_FACE_TOWARD_BLUE,ANGLE_TOWARD_RED, slowVelocity, slowAcceleration)
                .waitSeconds(.1)
                .splineToLinearHeading(CHAMBER_SLOT_FOUR,ANGLE_TOWARD_BLUE)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER));

    }

    public void intakeFirstTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(24, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_1, ANGLE_45_DEGREES)
                .splineToLinearHeading(
                        new Pose2d(RIGHT_OF_CHAMBER_INTAKE_1.position.x,
                                    RIGHT_OF_CHAMBER_INTAKE_1.position.y+6,
                                    ANGLE_TOWARD_BLUE), ANGLE_TOWARD_RED, slowVelocity)
                .setReversed(true)
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.SAMPLE_ACTUATOR_RETRCACT))
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .waitSeconds(.45)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE));
    }

    public void intakeSecondTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(5, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToLinearHeading(RIGHT_OF_CHAMBER_INTAKE_2, ANGLE_45_DEGREES)
                .splineToLinearHeading(
                        new Pose2d(RIGHT_OF_CHAMBER_INTAKE_2.position.x,
                                RIGHT_OF_CHAMBER_INTAKE_2.position.y+6,
                                ANGLE_TOWARD_BLUE), ANGLE_TOWARD_RED, slowVelocity)
                .setReversed(true)
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.SAMPLE_ACTUATOR_RETRCACT))
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE));

    }


    public void intakeThirdTeamSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(5, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .afterDisp(5, robotAdapter.getAction(RobotAdapter.ActionType.SAMPLE_ACTUATOR_RETRCACT))
                .splineToLinearHeading(OBS_ZONE_DUMP,ANGLE_TOWARD_RED)
                .stopAndAdd(robotAdapter.getAction(RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE));
    }
}



