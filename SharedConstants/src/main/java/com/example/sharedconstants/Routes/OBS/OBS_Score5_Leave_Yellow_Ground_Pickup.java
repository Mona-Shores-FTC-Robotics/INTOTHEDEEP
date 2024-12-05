package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_340_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.RobotAdapter.ActionType.DROP_OFF_GROUND_SAMPLE_WITHOUT_TRANSFER;
import static com.example.sharedconstants.RobotAdapter.ActionType.GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND;
import static com.example.sharedconstants.RobotAdapter.ActionType.PICKUP_FROM_GROUND_WITHOUT_TRANSFER;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class OBS_Score5_Leave_Yellow_Ground_Pickup extends Routes {

    public OBS_Score5_Leave_Yellow_Ground_Pickup(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_45_DEGREES);
        pickUpGroundSample(OBS_SPIKE_ONE);
        dropOffGroundSample();
//        pickUpGroundSample(OBS_SPIKE_TWO);
//        dropOffGroundSample();
//        pickUpGroundSample(OBS_SPIKE_THREE);
//        dropOffGroundSample();
//        pickupSpecimenFromTriangleComingFromThirdSpike();
//        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_ONE);
//        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
//        pickupSpecimenFromTriangle();
//        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_TWO);
//        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
//        pickupSpecimenFromTriangle();
//        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE);
//        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
//        pickupSpecimenFromTriangle();
//        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FOUR);
//        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
//        pickupSpecimenFromTriangle();
//        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pickUpGroundSample(Pose2d sampleLocation) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(3, robotAdapter.getAction(GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION))
                .strafeToLinearHeading(PoseToVector(sampleLocation).minus(new Vector2d(10,18)), Math.toRadians(45))
                .stopAndAdd(robotAdapter.getAction(PICKUP_FROM_GROUND_WITHOUT_TRANSFER))
                .waitSeconds(.5);
    }
    private void dropOffGroundSample() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .turnTo(Math.toRadians(280))
                .stopAndAdd(robotAdapter.getAction(DROP_OFF_GROUND_SAMPLE_WITHOUT_TRANSFER))
                .waitSeconds(.5);
    }

    public void pickupSecondGroundSample() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_TWO), ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_TWO), ANGLE_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_TWO), ANGLE_TOWARD_RED);
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

    public void pickupSpecimenFromTriangleComingFromThirdSpike() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_APPROACH), ANGLE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, slowVelocity, slowAcceleration);
    }


    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setTangent(ANGLE_315_DEGREES)
                .afterDisp(3, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_TRIANGLE_APPROACH), ANGLE_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP, ANGLE_TOWARD_RED, slowVelocity, slowAcceleration);
    }

}


