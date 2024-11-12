package com.example.sharedconstants.Routes.OBS.SampleFirst;

import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_BLUE;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.CHAMBER_SLOT_ONE;
import static com.example.sharedconstants.FieldConstants.NET_BASKET_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_BEHIND_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_DELIVER_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_ONE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_THREE;
import static com.example.sharedconstants.FieldConstants.OBS_SPIKE_TWO;
import static com.example.sharedconstants.FieldConstants.OBS_START_POSE_WITH_SAMPLE_PRELOAD;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static com.example.sharedconstants.FieldConstants.RIGHT_OF_CHAMBER;
import static com.example.sharedconstants.RobotAdapter.ActionType.DUMP_SAMPLE_IN_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HIGH_BASKET;
import static com.example.sharedconstants.RobotAdapter.ActionType.SAMPLE_LIFT_TO_HOME;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;

public class OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade extends Routes {
    public OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade(RobotAdapter robotAdapter) {
        super(robotAdapter);
    }

    public void buildRoute() {
        scorePreloadSampleInBasket();
        returnToStart();
        pushFirstNeutralSpecimen();
        pickupSpecimenFromWall(false);
        scoreOnHighChamber(CHAMBER_SLOT_ONE);
        observationBotRoute = obsTrajectoryActionBuilder.build();
    }

    public void scorePreloadSampleInBasket() {
        obsTrajectoryActionBuilder = robotAdapter.getActionBuilder(OBS_START_POSE_WITH_SAMPLE_PRELOAD)
                .waitSeconds(1)
                .strafeTo(PoseToVector(NET_BASKET_WALL))
                .afterDisp(3, robotAdapter.getAction(SAMPLE_LIFT_TO_HIGH_BASKET))
                .stopAndAdd(robotAdapter.getAction(DUMP_SAMPLE_IN_BASKET));
    }

    private void returnToStart() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .strafeTo(PoseToVector(OBS_START_POSE_WITH_SAMPLE_PRELOAD))
                .afterDisp(1, robotAdapter.getAction(SAMPLE_LIFT_TO_HOME));
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
                .splineToLinearHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(OBS_BEHIND_SPIKE_TWO), ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_TWO), ANGLE_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_TWO), ANGLE_TOWARD_RED);
    }

    public void pushThirdNeutralSpecimen() {
        obsTrajectoryActionBuilder = obsTrajectoryActionBuilder
                .splineToLinearHeading(RIGHT_OF_CHAMBER, ANGLE_TOWARD_BLUE)
                .splineToSplineHeading(OBS_BEHIND_SPIKE_THREE, ANGLE_TOWARD_OBSERVATION)
                .splineToConstantHeading(PoseToVector(OBS_SPIKE_THREE), ANGLE_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(OBS_DELIVER_SPIKE_THREE), ANGLE_TOWARD_RED);
    }

}
