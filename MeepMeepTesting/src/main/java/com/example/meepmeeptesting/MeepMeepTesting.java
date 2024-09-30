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

static RoutesToRun routeToRunSelection = RoutesToRun.PRELOAD; // here

    /** Set which robots should show up **/
    public static boolean SHOW_BLUE_AUDIENCE_BOT = true;
    public static boolean SHOW_BLUE_BACKSTAGE_BOT = true;
    public static boolean SHOW_RED_AUDIENCE_BOT = true;
    public static boolean SHOW_RED_BACKSTAGE_BOT = true;

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

        // Now each robot uses its own DriveShim instead of a general one
        if (SHOW_RED_AUDIENCE_BOT) {
            DriveShim redAudienceDriveShim = redAudienceBot.getDrive();
            RobotAdapter redAudienceAdapter = new MeepMeepDriveAdapter(redAudienceDriveShim);
            Routes redAudienceRoute = CreateRoute(redAudienceAdapter);
            redAudienceRoute.BuildRoutes();
            redAudienceBot.runAction(redAudienceRoute.getRedAudienceBotRoute());
        }

        if (SHOW_BLUE_BACKSTAGE_BOT) {
            DriveShim blueBackstageDriveShim = blueBackstageBot.getDrive();
            RobotAdapter blueBackstageAdapter = new MeepMeepDriveAdapter(blueBackstageDriveShim);
            Routes blueBackstageRoute = CreateRoute(blueBackstageAdapter);
            blueBackstageRoute.BuildRoutes();
            blueBackstageBot.runAction(blueBackstageRoute.getBlueBackstageBotRoute());
        }

        if (SHOW_BLUE_AUDIENCE_BOT) {
            DriveShim blueAudienceDriveShim = blueAudienceBot.getDrive();
            RobotAdapter blueAudienceAdapter = new MeepMeepDriveAdapter(blueAudienceDriveShim);
            Routes blueAudienceRoute = CreateRoute(blueAudienceAdapter);
            blueAudienceRoute.BuildRoutes();
            blueAudienceBot.runAction(blueAudienceRoute.getBlueAudienceBotRoute());
        }

        if (SHOW_RED_BACKSTAGE_BOT) {
            DriveShim redBackstageDriveShim = redBackstageBot.getDrive();
            RobotAdapter redBackstageAdapter = new MeepMeepDriveAdapter(redBackstageDriveShim);
            Routes redBackstageRoute = CreateRoute(redBackstageAdapter);
            redBackstageRoute.BuildRoutes();
            redBackstageBot.runAction(redBackstageRoute.getRedBackstageBotRoute());
        }

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

    public static Routes CreateRoute(RobotAdapter adapter) {
        if (routeToRunSelection == MeepMeepTesting.RoutesToRun.PRELOAD_AND_THREE_SPECIMENS) {
            return new Preload_and_Three_Specimens(adapter);
        } else if (routeToRunSelection == MeepMeepTesting.RoutesToRun.PRELOAD_AND_ONE_SPECIMEN) {
            return new Preload_and_One_Specimen(adapter);
        } else if (routeToRunSelection == MeepMeepTesting.RoutesToRun.PRELOAD_AND_ONE_SAMPLE) {
            return new Preload_and_One_Sample(adapter);
        } else if (routeToRunSelection == MeepMeepTesting.RoutesToRun.PRELOAD) {
            return new Preload(adapter);
        } else {
            return new BasicRoute(adapter);
        }
    }
}

