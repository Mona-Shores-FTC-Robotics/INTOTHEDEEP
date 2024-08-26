package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;
import com.example.sharedconstants.RobotDriveAdapter;


public abstract class Routes {
    protected RobotDriveAdapter roadRunnerDrive;

    public Routes(RobotDriveAdapter roadRunnerDrive) {
        this.roadRunnerDrive = roadRunnerDrive;
    }

    public abstract void BuildRoutes();

    // Variables to store routes for all start locations and team prop locations
    protected Action redAudienceBotTeamPropCenterRoute;
    protected Action redBackstageBotTeamPropCenterRoute;
    protected Action blueBackstageBotTeamPropCenterRoute;
    protected Action blueAudienceBotTeamPropCenterRoute;

    protected Action redBackstageBotTeamPropLeftRoute;
    protected Action blueAudienceBotTeamPropLeftRoute;
    protected Action redAudienceBotTeamPropLeftRoute;
    protected Action blueBackstageBotTeamPropLeftRoute;

    protected Action redBackstageBotTeamPropRightRoute;
    protected Action redAudienceBotTeamPropRightRoute;
    protected Action blueBackstageBotTeamPropRightRoute;
    protected Action blueAudienceBotTeamPropRightRoute;

    protected Action redAudienceBotRoute;

    public Action getRoute(AllianceColor allianceColor, SideOfField sideOfField, TeamPropLocation teamPropLocation) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        switch (teamPropLocation) {
                            case LEFT:
                                return getBlueAudienceBotTeamPropLeftRoute();
                            case CENTER:
                                return getBlueAudienceBotTeamPropCenterRoute();
                            case RIGHT:
                                return getBlueAudienceBotTeamPropRightRoute();
                        }
                        break;
                    case BACKSTAGE:
                        switch (teamPropLocation) {
                            case LEFT:
                                return getBlueBackstageBotTeamPropLeftRoute();
                            case CENTER:
                                return getBlueBackstageBotTeamPropCenterRoute();
                            case RIGHT:
                                return getBlueBackstageBotTeamPropRightRoute();
                        }
                        break;
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        switch (teamPropLocation) {
                            case LEFT:
                                return getRedAudienceBotTeamPropLeftRoute();
                            case CENTER:
                                return getRedAudienceBotTeamPropCenterRoute();
                            case RIGHT:
                                return getRedAudienceBotTeamPropRightRoute();
                        }
                        break;
                    case BACKSTAGE:
                        switch (teamPropLocation) {
                            case LEFT:
                                return getRedBackstageBotTeamPropLeftRoute();
                            case CENTER:
                                return getRedBackstageBotTeamPropCenterRoute();
                            case RIGHT:
                                return getRedBackstageBotTeamPropRightRoute();
                        }
                        break;
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    // Getter methods implemented in the base class
    public Action getBlueBackstageBotTeamPropLeftRoute() {
        return blueBackstageBotTeamPropLeftRoute;
    }

    public Action getBlueAudienceBotTeamPropLeftRoute() {
        return blueAudienceBotTeamPropLeftRoute;
    }

    public Action getBlueBackstageBotTeamPropCenterRoute() {
        return blueBackstageBotTeamPropCenterRoute;
    }

    public Action getBlueAudienceBotTeamPropCenterRoute() {
        return blueAudienceBotTeamPropCenterRoute;
    }

    public Action getBlueBackstageBotTeamPropRightRoute() {
        return blueBackstageBotTeamPropRightRoute;
    }

    public Action getBlueAudienceBotTeamPropRightRoute() {
        return blueAudienceBotTeamPropRightRoute;
    }

    public Action getRedBackstageBotTeamPropLeftRoute() {
        return redBackstageBotTeamPropLeftRoute;
    }

    public Action getRedAudienceBotTeamPropLeftRoute() {
        return redAudienceBotTeamPropLeftRoute;
    }

    public Action getRedBackstageBotTeamPropCenterRoute() {
        return redBackstageBotTeamPropCenterRoute;
    }

    public Action getRedAudienceBotTeamPropCenterRoute() {
        return redAudienceBotTeamPropCenterRoute;
    }

    public Action getRedBackstageBotTeamPropRightRoute() {
        return redBackstageBotTeamPropRightRoute;
    }

    public Action getRedAudienceBotTeamPropRightRoute() {
        return redAudienceBotTeamPropRightRoute;
    }

    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
    }
}