package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

// General-purpose adapter for robot actions
public interface RobotAdapter {

    // Enum for actions that both RealRobot and MeepMeep should handle
    enum ActionType {
        SAMPLE_INTAKE_ON,
        SAMPLE_INTAKE_OFF,
        SAMPLE_INTAKE_REVERSE,

        SPECIMEN_INTAKE_ON,
        SPECIMEN_INTAKE_OFF,
        SPECIMEN_INTAKE_REVERSE,

        SAMPLE_LIFT_TO_HOME,
        SAMPLE_LIFT_TO_HIGH_BASKET,
        SAMPLE_LIFT_TO_LOW_BASKET,

        GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND,
        GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION,
        DROP_OFF_GROUND_SAMPLE_WITHOUT_TRANSFER,
        PICKUP_FROM_GROUND,
        PREPARE_TO_SCORE_IN_HIGH_BASKET,
        PREPARE_TO_SCORE_IN_LOW_BASKET,
        DUMP_SAMPLE_IN_OBSERVATION_ZONE,

        MOVE_PRELOAD_SPECIMEN_TO_CW_HOME,
        GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL,
        HANG_SPECIMEN_ON_HIGH_CHAMBER,

        DISABLE_PRELOAD_MODE,

        LEVEL_1_ASCENT,
        LEVEL_2_ASCENT,
        LEVEL_3_ASCENT,

        SCORE_IN_BASKET,

        WAIT_FOR_SPECIMEN_INTAKE_FROM_WALL,
        SPECIMEN_PICKUP_RETRY, SIMPLE_DRIVE,
        SAMPLE_ACTUATOR_RETRACT,
        MOVE_ARM_IF_WE_MISSED,
        FLIP_UP_AND_RETRACT,
        DEPOWER_ARM,
        CONDITIONAL_PICKUP, CONDITIONAL_TRANSFER, PICKUP_FROM_GROUND_WITHOUT_TRANSFER

    }

    // Generalized method to get an Action based on a provided action type
    Action getAction(ActionType actionType);

    // The new method that will determine which builder to use
    TrajectoryActionBuilder getActionBuilder(Pose2d startPose);

    void setAllianceColor(FieldConstants.AllianceColor allianceColor);
    void setSideOfField(FieldConstants.SideOfField sideOfField);

}