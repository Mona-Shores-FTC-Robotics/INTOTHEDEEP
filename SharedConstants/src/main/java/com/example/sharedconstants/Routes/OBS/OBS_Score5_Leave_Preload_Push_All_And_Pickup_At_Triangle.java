package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_115_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FOUR;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class OBS_Score5_Leave_Preload_Push_All_And_Pickup_At_Triangle extends Routes {

    public OBS_Score5_Leave_Preload_Push_All_And_Pickup_At_Triangle(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    @Override
    public void buildRoute() {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(FieldConstants.OBS_START_POSE);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_45_DEGREES);
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        pushThirdNeutralSpecimen();
        pickupSpecimenFromTriangleComingFromThirdSpike();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_ONE);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_TWO);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_THREE);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FOUR);
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder.setTangent(ANGLE_315_DEGREES);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void pushFirstNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToSplineHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_ONE), ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_ONE), ANGLE_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_ONE), ANGLE_TOWARD_RED);
    }

    public void pushSecondNeutralSpecimen() {
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


