package com.example.sharedconstants.Routes.FunctionalRoutes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Pose2d;

public class PosesForFunctionalRoutesExample {
    public Pose2d startingPose;
    public Pose2d waypointPose;
    public Pose2d parkPose;
    public double parkOrientation;

    PosesForFunctionalRoutesExample(AllianceColor allianceColor, SideOfField sideOfField) {
        SetStartingPose(allianceColor, sideOfField);
        SetWaypointPose(allianceColor, sideOfField);
        SetParkPose(allianceColor);
    }

    private void SetStartingPose(AllianceColor allianceColor, SideOfField sideOfField) {
        if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.BACKSTAGE) {
            startingPose = BLUE_BACKSTAGE_START_POSE;
        } else if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.AUDIENCE) {
            startingPose = BLUE_AUDIENCE_START_POSE;
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.BACKSTAGE) {
            startingPose = RED_BACKSTAGE_START_POSE;
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.AUDIENCE) {
            startingPose = RED_AUDIENCE_START_POSE;
        }
    }

    private void SetWaypointPose(AllianceColor allianceColor, SideOfField sideOfField) {
        if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.AUDIENCE) {
            waypointPose = CHAMBER_RED_AUDIENCE;  // Default waypoint for BLUE AUDIENCE
        } else if (allianceColor == AllianceColor.BLUE && sideOfField == SideOfField.BACKSTAGE) {
            waypointPose = SPIKE_NEUTRAL_AUDIENCE_1; // Default waypoint for BLUE BACKSTAGE
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.AUDIENCE) {
            waypointPose = SPIKE_NEUTRAL_AUDIENCE_1;   // Default waypoint for RED AUDIENCE
        } else if (allianceColor == AllianceColor.RED && sideOfField == SideOfField.BACKSTAGE) {
            waypointPose = CHAMBER_RED_AUDIENCE;  // Default waypoint for RED BACKSTAGE
        }
    }

    private void SetParkPose(AllianceColor allianceColor) {
        if (allianceColor == AllianceColor.BLUE) {
            parkPose = CHAMBER_RED_AUDIENCE;
            parkOrientation = FACE_45_DEGREES;
        } else {
            parkPose = CHAMBER_RED_AUDIENCE;
            parkOrientation = FACE_315_DEGREES;
        }
    }
}
