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

        MOVE_PRELOAD_SPECIMEN_TO_CW_HOME,
        HANG_SPECIMEN_ON_HIGH_CHAMBER,

        INTAKE_SAMPLE_FROM_GROUND_AND_RETRACT,
        DEPOSIT_SAMPLE,
        DUMP_SAMPLE_IN_OBSERVATION_ZONE,

        LEVEL_1_ASCENT,
        LEVEL_2_ASCENT,
        LEVEL_3_ASCENT,

        GET_READY_FOR_INTAKE_FROM_GROUND,
        GET_READY_FOR_INTAKE_FROM_WALL
    }

    // Generalized method to get an Action based on a provided action type
    Action getAction(ActionType actionType);

    // The new method that will determine which builder to use
    TrajectoryActionBuilder getActionBuilder(Pose2d startPose);

    void setAllianceColor(FieldConstants.AllianceColor allianceColor);
    void setSideOfField(FieldConstants.SideOfField sideOfField);

}