package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepRobots.*;

import com.example.sharedconstants.Routes.FunctionalRoutes.FunctionalRoutesExample;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.DirectRoutes.DirectRoutesExample;
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
    public static RoutesToRun routesToRunSelection = RoutesToRun.FUNCTIONAL_ROUTES_EXAMPLE;

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = true;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = true;
    public static boolean SHOW_RED_AUDIENCE_BOT = true;
    public static boolean SHOW_RED_BACKSTAGE_BOT = true;

    public enum TeamPropLocation {LEFT, CENTER, RIGHT, ALL}
    enum RoutesToRun {DIRECT_ROUTES_EXAMPLE, FUNCTIONAL_ROUTES_EXAMPLE}

    public static void main(String[] args) {

        //Set the window Size for MeepMeep
        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MeepMeepRobots.createRobots(meepMeep);

        //This sets the drive for a "dummy robot"
        DriveShim driveShim = MeepMeepRobots.roadRunnerBot.getDrive();

        // Use the adapter
        RobotDriveAdapter robotDriveAdapter = new MeepMeepDriveAdapter(driveShim);

        // Create a Routes instance based on the selected route
        Routes routes;

        if (routesToRunSelection == RoutesToRun.DIRECT_ROUTES_EXAMPLE) {
            routes = new DirectRoutesExample(robotDriveAdapter);
        } else { //if(routesToRunSelection == RoutesToRun.FUNCTIONAL_ROUTES_EXAMPLE){
            routes = new FunctionalRoutesExample(robotDriveAdapter);
        }

        // Build the routes
        routes.BuildRoutes();

        // Set the routes for team prop(s)
        switch (teamPropLocation) {
            case LEFT:
                MeepMeepRobots.setTeamPropLeftRoutes(routes);
                break;
            case CENTER:
                MeepMeepRobots.setTeamPropCenterRoutes(routes);
                break;
            case RIGHT:
                MeepMeepRobots.setTeamPropRightRoutes(routes);
                break;
            case ALL:
                MeepMeepRobots.setTeamPropAllRoutes(routes);
                break;
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

