package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepRobots.*;

import com.example.sharedconstants.Routes.RoutesBasic;
import com.example.sharedconstants.Routes.RoutesSpikeBackdropPark;
import com.example.sharedconstants.RobotDriveAdapter;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    /**
     * SET TO PICK WHICH ROUTES TO RUN:
     * Prop Location: LEFT, RIGHT, OR CENTER
     * Routes:
     *      SPIKE_BACKDROP_PARK,
     **/

    public static TeamPropLocation teamPropLocation = TeamPropLocation.ALL;
    public static RoutesToRun routesToRunSelection = RoutesToRun.BASIC;

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = true;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = true;
    public static boolean SHOW_RED_AUDIENCE_BOT = true;
    public static boolean SHOW_RED_BACKSTAGE_BOT = true;

    public enum TeamPropLocation {LEFT, CENTER, RIGHT, ALL}
    enum RoutesToRun {SPIKE_BACKDROP_PARK, BASIC}

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MeepMeepRobots.createRobots(meepMeep);

        //This sets the drive for a "dummy robot"
        DriveShim driveShim = MeepMeepRobots.roadRunnerBot.getDrive();

        // Use the adapter
        RobotDriveAdapter robotDriveAdapter = new MeepMeepDriveAdapter(driveShim);

        //Set the routes that will be run
        if (routesToRunSelection == RoutesToRun.SPIKE_BACKDROP_PARK) {
            RoutesSpikeBackdropPark routesSpikeBackdropPark = new RoutesSpikeBackdropPark(robotDriveAdapter);
            routesSpikeBackdropPark.BuildRoutes();
            if (teamPropLocation == TeamPropLocation.LEFT)
                MeepMeepRobots.setTeamPropLeftRoutes(
                        routesSpikeBackdropPark.blueBackstageBotTeamPropLeftRoute,
                        routesSpikeBackdropPark.blueAudienceBotTeamPropLeftRoute,
                        routesSpikeBackdropPark.redBackstageBotTeamPropLeftRoute,
                        routesSpikeBackdropPark.redAudienceBotTeamPropLeftRoute);
            if (teamPropLocation == TeamPropLocation.CENTER)
                MeepMeepRobots.setTeamPropCenterRoutes(
                        routesSpikeBackdropPark.blueBackstageBotTeamPropCenterRoute,
                        routesSpikeBackdropPark.blueAudienceBotTeamPropCenterRoute,
                        routesSpikeBackdropPark.redBackstageBotTeamPropCenterRoute,
                        routesSpikeBackdropPark.redAudienceBotTeamPropCenterRoute);
            if (teamPropLocation == TeamPropLocation.RIGHT)
                MeepMeepRobots.setTeamPropCenterRoutes(
                        routesSpikeBackdropPark.blueBackstageBotTeamPropRightRoute,
                        routesSpikeBackdropPark.blueAudienceBotTeamPropRightRoute,
                        routesSpikeBackdropPark.redBackstageBotTeamPropRightRoute,
                        routesSpikeBackdropPark.redAudienceBotTeamPropRightRoute);
            if (teamPropLocation == TeamPropLocation.ALL)
                MeepMeepRobots.setTeamPropAllRoutes(
                        routesSpikeBackdropPark.blueBackstageBotTeamPropLeftRoute, routesSpikeBackdropPark.blueBackstageBotTeamPropCenterRoute, routesSpikeBackdropPark.blueBackstageBotTeamPropRightRoute,
                        routesSpikeBackdropPark.blueAudienceBotTeamPropCenterRoute, routesSpikeBackdropPark.blueAudienceBotTeamPropLeftRoute, routesSpikeBackdropPark.blueAudienceBotTeamPropRightRoute,
                        routesSpikeBackdropPark.redBackstageBotTeamPropCenterRoute, routesSpikeBackdropPark.redBackstageBotTeamPropLeftRoute, routesSpikeBackdropPark.redBackstageBotTeamPropRightRoute,
                        routesSpikeBackdropPark.redAudienceBotTeamPropCenterRoute, routesSpikeBackdropPark.redAudienceBotTeamPropLeftRoute, routesSpikeBackdropPark.redAudienceBotTeamPropRightRoute);
        }

        if (routesToRunSelection == RoutesToRun.BASIC) {
            RoutesBasic routesBasic = new RoutesBasic(robotDriveAdapter);
            routesBasic.BuildRoutes();
            if (teamPropLocation == TeamPropLocation.LEFT)
                MeepMeepRobots.setTeamPropLeftRoutes(
                        routesBasic.blueBackstageBotTeamPropLeftRoute,
                        routesBasic.blueAudienceBotTeamPropLeftRoute,
                        routesBasic.redBackstageBotTeamPropLeftRoute,
                        routesBasic.redAudienceBotTeamPropLeftRoute);
            if (teamPropLocation == TeamPropLocation.CENTER)
                MeepMeepRobots.setTeamPropCenterRoutes(
                        routesBasic.blueBackstageBotTeamPropCenterRoute,
                        routesBasic.blueAudienceBotTeamPropCenterRoute,
                        routesBasic.redBackstageBotTeamPropCenterRoute,
                        routesBasic.redAudienceBotTeamPropCenterRoute);
            if (teamPropLocation == TeamPropLocation.RIGHT)
                MeepMeepRobots.setTeamPropCenterRoutes(
                        routesBasic.blueBackstageBotTeamPropRightRoute,
                        routesBasic.blueAudienceBotTeamPropRightRoute,
                        routesBasic.redBackstageBotTeamPropRightRoute,
                        routesBasic.redAudienceBotTeamPropRightRoute);
            if (teamPropLocation == TeamPropLocation.ALL)
                MeepMeepRobots.setTeamPropAllRoutes(
                        routesBasic.blueBackstageBotTeamPropLeftRoute, routesBasic.blueBackstageBotTeamPropCenterRoute, routesBasic.blueBackstageBotTeamPropRightRoute,
                        routesBasic.blueAudienceBotTeamPropCenterRoute, routesBasic.blueAudienceBotTeamPropLeftRoute, routesBasic.blueAudienceBotTeamPropRightRoute,
                        routesBasic.redBackstageBotTeamPropCenterRoute, routesBasic.redBackstageBotTeamPropLeftRoute, routesBasic.redBackstageBotTeamPropRightRoute,
                        routesBasic.redAudienceBotTeamPropCenterRoute, routesBasic.redAudienceBotTeamPropLeftRoute, routesBasic.redAudienceBotTeamPropRightRoute);
        }
        addRobotsToField(meepMeep);
    }

    private static void addRobotsToField(MeepMeep meepMeep_local) {
        if (teamPropLocation != TeamPropLocation.ALL) {
            if (SHOW_BLUE_AUDIENCE_BOT) meepMeep_local.addEntity(blueAudienceBot);
            if (SHOW_BLUE_BACKSTAGE_BOT) meepMeep_local.addEntity(blueBackstageBot);
            if (SHOW_RED_AUDIENCE_BOT) meepMeep_local.addEntity(redAudienceBot);
            if (SHOW_RED_BACKSTAGE_BOT) meepMeep_local.addEntity(redBackstageBot);
        }
        if (teamPropLocation == TeamPropLocation.ALL)
        {
            if (SHOW_BLUE_BACKSTAGE_BOT){
                meepMeep_local.addEntity(blueBackstageBot);
                meepMeep_local.addEntity(blueBackstageBotLeft);
                meepMeep_local.addEntity(blueBackstageBotRight);
            }
            if (SHOW_BLUE_AUDIENCE_BOT){
                meepMeep_local.addEntity(blueAudienceBot);
                meepMeep_local.addEntity(blueAudienceBotLeft);
                meepMeep_local.addEntity(blueAudienceBotRight);
            }
            if (SHOW_RED_BACKSTAGE_BOT){
                meepMeep_local.addEntity(redBackstageBot);
                meepMeep_local.addEntity(redBackstageBotLeft);
                meepMeep_local.addEntity(redBackstageBotRight);
            }
            if (SHOW_RED_AUDIENCE_BOT){
                meepMeep_local.addEntity(redAudienceBot);
                meepMeep_local.addEntity(redAudienceBotLeft);
                meepMeep_local.addEntity(redAudienceBotRight);
            }
        }

        String filePath = "CenterstageRotated.png";
        System.out.println(new File(".").getAbsolutePath());
        Image img = null;
        try {
            img = ImageIO.read(new File(filePath));
        } catch (IOException e) {
        }


        meepMeep_local.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(.95f)
                .start();

    }

}

