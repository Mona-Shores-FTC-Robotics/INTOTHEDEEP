package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD_AND_ONE_SPECIMEN;
import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SideOfField.AUDIENCE;
import static com.example.sharedconstants.FieldConstants.SideOfField.BACKSTAGE;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.BasicRoute;
import com.example.sharedconstants.Routes.Preload;
import com.example.sharedconstants.Routes.Preload_and_One_Sample;
import com.example.sharedconstants.Routes.Preload_and_One_Specimen;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.Preload_and_Three_Specimens;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    static RoutesToRun RED_BACKSTAGE_ROUTE = PRELOAD; // here
    static RoutesToRun BLUE_AUDIENCE_ROUTE = PRELOAD_AND_ONE_SPECIMEN; // here

    static RoutesToRun RED_AUDIENCE_ROUTE = PRELOAD; // here
    static RoutesToRun BLUE_BACKSTAGE_ROUTE = PRELOAD; // here

    enum RoutesToRun {
        BASIC,
        PRELOAD_AND_THREE_SPECIMENS,
        PRELOAD,
        PRELOAD_AND_ONE_SPECIMEN,
        PRELOAD_AND_ONE_SAMPLE} // here

    // Store references to AdaptedBots
    private static AdaptedBot redAudienceBot;
    private static AdaptedBot blueBackstageBot;
    private static AdaptedBot blueAudienceBot;
    private static AdaptedBot redBackstageBot;

    public static void main(String[] args) {

        //Set the window Size for MeepMeep
        MeepMeep meepMeep = new MeepMeep(800);

        // Create the robots dynamically and configure them
        if (RED_AUDIENCE_ROUTE!=null) {
            redAudienceBot = createAdaptedBotAndRunRoute(meepMeep, RED, AUDIENCE, new ColorSchemeRedDark(), FieldConstants.RED_AUDIENCE_START_POSE, RED_AUDIENCE_ROUTE);
        }
        if (BLUE_BACKSTAGE_ROUTE!=null) {
            blueBackstageBot = createAdaptedBotAndRunRoute(meepMeep, BLUE, BACKSTAGE,  new ColorSchemeBlueDark(), FieldConstants.BLUE_AUDIENCE_START_POSE, RED_BACKSTAGE_ROUTE);
        }
        if (BLUE_AUDIENCE_ROUTE != null) {
            blueAudienceBot = createAdaptedBotAndRunRoute(meepMeep, BLUE, AUDIENCE, new ColorSchemeBlueLight(), FieldConstants.BLUE_AUDIENCE_START_POSE, BLUE_AUDIENCE_ROUTE);
        }
        if (RED_BACKSTAGE_ROUTE != null) {
            redBackstageBot = createAdaptedBotAndRunRoute(meepMeep, RED, BACKSTAGE, new ColorSchemeRedLight(), FieldConstants.RED_BACKSTAGE_START_POSE, RED_BACKSTAGE_ROUTE);
        }

        startMeepMeep(meepMeep);
    }
    // Create an AdaptedBot and run the selected route
    private static AdaptedBot createAdaptedBotAndRunRoute(MeepMeep meepMeep,
                                                          FieldConstants.AllianceColor allianceColor,
                                                          FieldConstants.SideOfField sideOfField,
                                                          ColorScheme colorScheme,
                                                          Pose2d startPose,
                                                          RoutesToRun selectedRoute) {
        // Create the adapted bot
        AdaptedBot adaptedBot = new AdaptedBot(meepMeep, colorScheme, startPose);

        // Set the alliance color in the adapter
        adaptedBot.setAllianceColor(allianceColor);
        adaptedBot.setSideOfField(sideOfField);

        // Create and build the route based on the selected route
        adaptedBot.setRoute(MeepMeepTesting.createRoute(adaptedBot.getAdapter(), selectedRoute));

        // Run the route on the bot
        adaptedBot.runAction(adaptedBot.getRoute().getRouteAction(allianceColor, sideOfField));

        // Add the bot to the MeepMeep field
        meepMeep.addEntity(adaptedBot.getBot());

        return adaptedBot;
    }


    // Helper function to determine which side of the field a pose is located
    private static FieldConstants.SideOfField getSideOfField(Pose2d pose) {
        if (pose == FieldConstants.RED_AUDIENCE_START_POSE || pose == FieldConstants.BLUE_AUDIENCE_START_POSE) {
            return AUDIENCE;
        } else {
            return FieldConstants.SideOfField.BACKSTAGE;
        }
    }

    // Start MeepMeep with custom settings
    private static void startMeepMeep(MeepMeep meepMeep_local) {
        String filePath = "intothedeep2.png";  // Customize the field background

        try {
            Image img = ImageIO.read(new File(filePath));
            meepMeep_local.setBackground(img);
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep_local.setDarkMode(false)
                .setBackgroundAlpha(.95f)
                .start();
    }

    // Helper method to create the route based on the route selection
    public static Routes createRoute(RobotAdapter adapter, RoutesToRun routeToRunSelection) {
        switch (routeToRunSelection) {
            case PRELOAD_AND_THREE_SPECIMENS:
                return new Preload_and_Three_Specimens(adapter);
            case PRELOAD_AND_ONE_SPECIMEN:
                return new Preload_and_One_Specimen(adapter);
            case PRELOAD_AND_ONE_SAMPLE:
                return new Preload_and_One_Sample(adapter);
            case PRELOAD:
                return new Preload(adapter);
            default:
                return new BasicRoute(adapter);
        }
    }
}

