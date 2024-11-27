package com.example.sharedconstants.Routes.OBS;

import static com.example.sharedconstants.FieldConstants.ANGLE_135_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_340_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_FIVE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_NINE_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_SEVEN_REDO;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_APPROACH_ALLIANCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_PICKUP_ALLIANCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_APPROACH_REDO;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP;
import static com.example.sharedconstants.FieldConstants.OBS_TRIANGLE_PICKUP_REDO;
import static com.example.sharedconstants.FieldConstants.PICKUP_ROOM;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER_REDO;
import static com.example.sharedconstants.RobotAdapter.ActionType.HANG_SPECIMEN_ON_HIGH_CHAMBER;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.RobotAdapter;

public class OBS_Score5_Redo_LINE_TO extends OBS_Score_1_Specimen_Preload_REDO {

    public OBS_Score5_Redo_LINE_TO(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }
    public void buildRoute(){
        super.buildRoute();
        pushFirstNeutralSpecimen();
        pushSecondNeutralSpecimen();
        pushThirdNeutralSpecimen();
        pickupSpecimenFromCornerAllianceWall();
        scoreOnHighChamberFromCorner(CHAMBER_SLOT_THREE_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_FIVE_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_SEVEN_REDO);
        pickupSpecimenFromTriangle();
        scoreOnHighChamberFromTriangle(CHAMBER_SLOT_NINE_REDO);
        driveToPark();
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    private void pickupSpecimenFromCornerAllianceWall() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(0, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .splineToConstantHeading(PoseToVector(OBS_CORNER_APPROACH_ALLIANCE_WALL), ANGLE_TOWARD_RED, obsFastVelocity)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(OBS_CORNER_PICKUP_ALLIANCE_WALL), ANGLE_TOWARD_RED, obsSlowVelocity)
                .waitSeconds(.125);
    }

    public void pushFirstNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RIGHT_OF_CHAMBER_REDO), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_ONE_REDO),ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .lineToY(OBS_DELIVER_SPIKE_ONE_REDO.position.y, obsFastVelocity)
                .setReversed(false)
                .lineToY(OBS_SPIKE_ONE_REDO.position.y, obsFastVelocity);
    }

    public void pushSecondNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_TWO_REDO), ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .lineToY(OBS_DELIVER_SPIKE_TWO_REDO.position.y, obsFastVelocity)
                .setReversed(false)
                .lineToY(OBS_SPIKE_TWO_REDO.position.y, obsFastVelocity);
    }

    private void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_THREE_REDO), ANGLE_TOWARD_OBSERVATION, obsFastVelocity)
                .setReversed(true)
                .lineToY(OBS_DELIVER_SPIKE_THREE_REDO.position.y, obsFastVelocity);
    }


    public void scoreOnHighChamberFromCorner(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-5)), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }

    public void scoreOnHighChamberFromTriangle(Pose2d chamberSlot) {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot).plus(new Vector2d(0,-5)), ANGLE_TOWARD_BLUE, obsFastVelocity)
                .setReversed(false)
                .splineToConstantHeading(PoseToVector(chamberSlot), ANGLE_TOWARD_BLUE, obsSlowVelocity)
                .stopAndAdd(robotAdapter.getAction(HANG_SPECIMEN_ON_HIGH_CHAMBER));
    }


    public void pickupSpecimenFromTriangle() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .afterDisp(6, robotAdapter.getAction(RobotAdapter.ActionType.GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL))
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_APPROACH_REDO, ANGLE_TOWARD_RED, obsFastVelocity)
                .setReversed(true)
                .splineToLinearHeading(OBS_TRIANGLE_PICKUP_REDO, ANGLE_TOWARD_RED, obsSlowVelocity)
                .waitSeconds(.125);
    }

    private void driveToPark() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(PoseToVector(OBS_TRIANGLE_PICKUP), ANGLE_135_DEGREES), ANGLE_340_DEGREES, obsFastVelocity);
    }
}

