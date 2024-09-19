package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepRobots.*;

import com.example.sharedconstants.Routes.BasicRoute;
import com.example.sharedconstants.Routes.RRPathGenExample;
import com.example.sharedconstants.Routes.FunctionalRoutes.FunctionalRoutesExample;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.DirectRoutesExample;
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

    //TODO Naming convention
    // discuss how to name our OpModes, does this make sense: AUD_2_1_BACK_0_4

    public static RoutesToRun routesToRunSelection = RoutesToRun.A1Spec_3Samp_B4Spec_0Samp; // here

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = false;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = false;
    public static boolean SHOW_RED_AUDIENCE_BOT = true;
    public static boolean SHOW_RED_BACKSTAGE_BOT = true;

    public enum TeamPropLocation {LEFT, CENTER, RIGHT, ALL, NONE}
    enum RoutesToRun {A1Spec_3Samp_B4Spec_0Samp, FUNCTIONAL_ROUTES_EXAMPLE, RRPATHGEN, BASIC_ROUTE} // here

    public static void main(String[] args) {

        //Set the window Size for MeepMeep
        MeepMeep meepMeep = new MeepMeep(1400);

        //This method makes 4 robots (2 red robots and 2 blue robots)
        MeepMeepRobots.createRobots(meepMeep);

        //This sets the drive for a "dummy robot"
        DriveShim driveShim = MeepMeepRobots.roadRunnerBot.getDrive();

        // Use the adapter
        RobotDriveAdapter robotDriveAdapter = new MeepMeepDriveAdapter(driveShim);

        // Create a Routes instance based on the selected route
        Routes routes;

        if (routesToRunSelection == RoutesToRun.A1Spec_3Samp_B4Spec_0Samp) { // here
            routes = new DirectRoutesExample(robotDriveAdapter);
        } else if (routesToRunSelection == RoutesToRun.RRPATHGEN){
            routes = new RRPathGenExample(robotDriveAdapter);
        } else if (routesToRunSelection == RoutesToRun.BASIC_ROUTE) {
            routes = new BasicRoute(robotDriveAdapter);
        } else { //if(routesToRunSelection == RoutesToRun.FUNCTIONAL_ROUTES_EXAMPLE){
            routes = new FunctionalRoutesExample(robotDriveAdapter);
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

