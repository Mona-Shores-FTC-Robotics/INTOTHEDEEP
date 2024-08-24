package com.example.sharedconstants.Routes.FunctionalRoutes;

import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Pose2d;

public class PosesForFunctionalRoutesExample {
    public Pose2d startingPose;
    public Pose2d waypointPose;
    public Pose2d parkPose;
    public double parkOrientation;

    PosesForFunctionalRoutesExample(AllianceColor allianceColor, SideOfField sideOfField, TeamPropLocation teamPropLocation) {
        SetStartingPose(allianceColor, sideOfField);
        SetWaypointPose(allianceColor, sideOfField, teamPropLocation);
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

    public void SetWaypointPose(AllianceColor allianceColor, SideOfField sideOfField, TeamPropLocation teamPropLocation) {
        if (allianceColor == AllianceColor.BLUE) {
            switch (teamPropLocation) {
                case LEFT: {
                    if (sideOfField == SideOfField.AUDIENCE) {
                        waypointPose = BLUE_AUDIENCE_SPIKE_L;
                    } else waypointPose = BLUE_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    if (sideOfField == SideOfField.AUDIENCE) {
                        waypointPose = BLUE_AUDIENCE_SPIKE_R;
                    } else waypointPose = BLUE_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER: {
                    if (sideOfField == SideOfField.AUDIENCE) {
                        waypointPose = BLUE_AUDIENCE_SPIKE_C;
                    } else waypointPose = BLUE_BACKSTAGE_SPIKE_C;
                    break;
                }
            }
        } else {
            switch (teamPropLocation) {
                case LEFT: {
                    if (sideOfField == SideOfField.AUDIENCE) {
                        waypointPose = RED_AUDIENCE_SPIKE_L;
                    } else waypointPose = RED_BACKSTAGE_SPIKE_L;
                    break;
                }
                case RIGHT: {
                    if (sideOfField == SideOfField.AUDIENCE) {
                        waypointPose = RED_AUDIENCE_SPIKE_R;
                    } else waypointPose = RED_BACKSTAGE_SPIKE_R;
                    break;
                }
                case CENTER:
                default: {
                    if (sideOfField == SideOfField.AUDIENCE) {
                        waypointPose = RED_AUDIENCE_SPIKE_C;
                    } else waypointPose = RED_BACKSTAGE_SPIKE_C;
                    break;
                }
            }
        }
    }

    public void SetParkPose(AllianceColor allianceColor) {
        if (allianceColor == AllianceColor.BLUE) {
            parkPose = BLUE_BACKSTAGE_PARK_LANE_C;
            parkOrientation = FACE_45_DEGREES;
        } else {
            parkPose = RED_BACKSTAGE_PARK_LANE_D;
            parkOrientation = FACE_315_DEGREES;
        }
    }

}
