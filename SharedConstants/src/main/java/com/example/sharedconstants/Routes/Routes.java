package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotDriveAdapter;

import java.lang.reflect.Field;


public abstract class Routes {
    protected RobotDriveAdapter roadRunnerDrive;

    public Routes(RobotDriveAdapter roadRunnerDrive) {
        this.roadRunnerDrive = roadRunnerDrive;
    }

    public abstract void BuildRoutes();

    // Variables to store routes for all start locations and team prop locations
    protected Action redAudienceBotRoute;
    protected Action redBackstageBotRoute;
    protected Action blueBackstageBotRoute;
    protected Action blueAudienceBotRoute;

    // These are the defaults
    protected Pose2d redAudienceStartPose = RED_AUDIENCE_START_POSE;
    protected Pose2d redBackstageStartPose = RED_BACKSTAGE_START_POSE;
    protected Pose2d blueBackstageStartPose = BLUE_BACKSTAGE_START_POSE;
    protected Pose2d blueAudienceStartPose = BLUE_AUDIENCE_START_POSE;

    // Method to return the corresponding starting pose
    public Pose2d getStartingPose(AllianceColor allianceColor, SideOfField sideOfField) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        return blueAudienceStartPose;
                    case BACKSTAGE:
                        return blueBackstageStartPose;
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        return redAudienceStartPose;
                    case BACKSTAGE:
                        return redBackstageStartPose;
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    public Action getRoute(AllianceColor allianceColor, SideOfField sideOfField) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        return getBlueAudienceBotRoute();  // Consolidated method for BLUE AUDIENCE
                    case BACKSTAGE:
                        return getBlueBackstageBotRoute();  // Consolidated method for BLUE BACKSTAGE
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        return getRedAudienceBotRoute();  // Consolidated method for RED AUDIENCE
                    case BACKSTAGE:
                        return getRedBackstageBotRoute();  // Consolidated method for RED BACKSTAGE
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
    }

    public Action getBlueAudienceBotRoute() {
        return blueAudienceBotRoute;
    }

    public Action getBlueBackstageBotRoute() {
        return blueBackstageBotRoute;
    }

    public Action getRedBackstageBotRoute() {
        return redBackstageBotRoute;
    }
}