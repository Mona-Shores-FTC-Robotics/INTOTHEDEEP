package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.SPIKE_1_GROUND_PICKUP;
import static com.example.sharedconstants.FieldConstants.SPIKE_2_GROUND_PICKUP;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;

public class OBS_Score5_Preload_Ground_Pickup_And_Dump_And_Pickup_At_Triangle extends OBS_Score_1_Specimen_Preload {

    public OBS_Score5_Preload_Ground_Pickup_And_Dump_And_Pickup_At_Triangle(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }


    @Override
    public void buildRoute() {
        super.buildRoute();
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
        intakeAndDumpGroundSample(SPIKE_1_GROUND_PICKUP , ANGLE_TOWARD_OBSERVATION);
        intakeAndDumpGroundSample(SPIKE_2_GROUND_PICKUP , ANGLE_TOWARD_OBSERVATION);
        pushThirdNeutralSpecimen();
        pickupSpecimenFromTriangleComingFromThirdSpike();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_TWO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FOUR);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE);

        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void intakeAndDumpGroundSample(Pose2d groundSamplePose, Double approachTangent) {

        Vector2d pickupVector = new Vector2d(groundSamplePose.position.x, groundSamplePose.position.y+4);
        Vector2d dumpVector = new Vector2d(groundSamplePose.position.x, groundSamplePose.position.y-4);

        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(24, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND))
                .splineToConstantHeading(PoseToVector(groundSamplePose), approachTangent)
                .splineToConstantHeading(pickupVector, ANGLE_TOWARD_RED)
                .waitSeconds(.1)
                .setReversed(true)
                //If we don't pick up the ground sample, then we have to retract
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.SAMPLE_ACTUATOR_RETRACT))
                .afterDisp(6, robotAdapter.getAction(RobotAdapter.ActionType.DUMP_SAMPLE_IN_OBSERVATION_ZONE))
                .splineToConstantHeading(dumpVector,ANGLE_TOWARD_RED)
                .waitSeconds(.1);
   }

    public void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToSplineHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_THREE), ANGLE_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_THREE), ANGLE_TOWARD_RED);
    }

    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_115_DEGREES)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }


    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_APPROACH), ANGLE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, slowVelocity, slowAcceleration);
    }

    public void pickupSpecimenFromTriangleComingFromThirdSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_APPROACH), ANGLE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, slowVelocity, slowAcceleration);
    }

}


