package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepRobots.*;

import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.BasicRoute;
import com.example.sharedconstants.Routes.Preload;
import com.example.sharedconstants.Routes.Preload_and_One_Sample;
import com.example.sharedconstants.Routes.Preload_and_One_Specimen;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.Preload_and_Three_Specimens;
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

    static RoutesToRun routesToRunSelection = RoutesToRun.PRELOAD_AND_THREE_SPECIMENS; // here

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = false;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = false;
    public static boolean SHOW_RED_AUDIENCE_BOT = true;
    public static boolean SHOW_RED_BACKSTAGE_BOT = true;

    public enum TeamPropLocation {LEFT, CENTER, RIGHT, ALL, NONE}
    enum RoutesToRun {
        BASIC,
        PRELOAD_AND_THREE_SPECIMENS,
        PRELOAD,
        PRELOAD_AND_ONE_SPECIMEN,
        PRELOAD_AND_ONE_SAMPLE} // here

    public static void main(String[] args) {

        //Set the window Size for MeepMeep
        MeepMeep meepMeep = new MeepMeep(800);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MeepMeepRobots.createRobots(meepMeep);

        //This sets the drive for a "dummy robot"
        DriveShim driveShim = MeepMeepRobots.roadRunnerBot.getDrive();

        // Use the adapter
        RobotAdapter robotAdapter = new MeepMeepDriveAdapter(driveShim);

        // Create a Routes instance based on the selected route
        Routes routes;

        if (routesToRunSelection == RoutesToRun.PRELOAD_AND_THREE_SPECIMENS) { // here
            routes = new Preload_and_Three_Specimens(robotAdapter);
        } else if (routesToRunSelection == RoutesToRun.PRELOAD_AND_ONE_SPECIMEN){
            routes = new Preload_and_One_Specimen(robotAdapter);
        } else if (routesToRunSelection == RoutesToRun.PRELOAD_AND_ONE_SAMPLE) {
            routes = new Preload_and_One_Sample(robotAdapter);
        } else if (routesToRunSelection == RoutesToRun.PRELOAD){
            routes = new Preload(robotAdapter);
        } else { //if(routesToRunSelection == RoutesToRun.BASIC){
            routes = new BasicRoute(robotAdapter);
        }

        // Build the routes
        routes.BuildRoutes();

        MeepMeepRobots.setRoutes(routes);

        addRobotsToField(meepMeep);
    }

    private static void addRobotsToField(MeepMeep meepMeep_local) {

        if (SHOW_BLUE_AUDIENCE_BOT) meepMeep_local.addEntity(blueAudienceBot);
        if (SHOW_BLUE_BACKSTAGE_BOT) meepMeep_local.addEntity(blueBackstageBot);
        if (SHOW_RED_AUDIENCE_BOT) meepMeep_local.addEntity(redAudienceBot);
        if (SHOW_RED_BACKSTAGE_BOT) meepMeep_local.addEntity(redBackstageBot);

//        String filePath = "intothedeep1.png";
        String filePath = "intothedeep2.png";
//        String filePath = "intothedeep3.png";
        Image img = null;
        try {
            img = ImageIO.read(new File(filePath));
        } catch (IOException e) {
            e.printStackTrace();  // Log the exception details
        }

        meepMeep_local.setBackground(img)
                .setDarkMode(false)
                .setBackgroundAlpha(.95f)
                .start();
    }
}

