package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.BASIC;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD_AND_ONE_SAMPLE;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD_AND_ONE_SPECIMEN;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD_AND_THREE_SPECIMENS;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.PRELOAD_AND_TWO_SPECIMENS;
import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SideOfField.NET;
import static com.example.sharedconstants.FieldConstants.SideOfField.OBSERVATION;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.BasicRoute;
import com.example.sharedconstants.Routes.OBS_Preload_and_Two_Specimens;
import com.example.sharedconstants.Routes.Preload;
import com.example.sharedconstants.Routes.NET_Preload_and_One_Sample;
import com.example.sharedconstants.Routes.OBS_Preload_and_One_Specimen;
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

    static RoutesToRun redObservationRoute = PRELOAD_AND_TWO_SPECIMENS; // here
    static RoutesToRun blueObservationRoute = PRELOAD_AND_TWO_SPECIMENS; // here
    static RoutesToRun redNetRoute =PRELOAD_AND_ONE_SAMPLE; // here
    static RoutesToRun blueNetRoute = PRELOAD_AND_ONE_SAMPLE; // here

    enum RoutesToRun {
        BASIC,
        PRELOAD_AND_THREE_SPECIMENS,
        PRELOAD_AND_TWO_SPECIMENS,
        PRELOAD,
        PRELOAD_AND_ONE_SPECIMEN,
        PRELOAD_AND_ONE_SAMPLE} // here

    public static void main(String[] args) {

        //Set the window Size for MeepMeep
        MeepMeep meepMeep = new MeepMeep(800);

        // Create the robots dynamically and configure them
        if (redNetRoute != null) {
            createAdaptedBotAndRunRoute(meepMeep, RED, NET, new ColorSchemeRedDark(), redNetRoute);
        }
        if (redObservationRoute != null) {
            createAdaptedBotAndRunRoute(meepMeep, RED, OBSERVATION, new ColorSchemeRedLight(), redObservationRoute);
        }
        if (blueNetRoute != null) {
            createAdaptedBotAndRunRoute(meepMeep, BLUE, NET, new ColorSchemeBlueDark(), blueNetRoute);
        }
        if (blueObservationRoute != null) {
            createAdaptedBotAndRunRoute(meepMeep, BLUE, OBSERVATION, new ColorSchemeBlueLight(), blueObservationRoute);
        }

        startMeepMeep(meepMeep);
    }
    // Create an AdaptedBot and run the selected route
    private static void createAdaptedBotAndRunRoute(MeepMeep meepMeep,
                                                    FieldConstants.AllianceColor allianceColor,
                                                    FieldConstants.SideOfField sideOfField,
                                                    ColorScheme colorScheme,
                                                    RoutesToRun selectedRoute) {

        // Create the adapted bot
        MeepMeepBot meepMeepBot = new MeepMeepBot(
                meepMeep, colorScheme, allianceColor, sideOfField); // Default start

        meepMeepBot.getAdapter().setSideOfField(sideOfField);
        meepMeepBot.getAdapter().setAllianceColor(allianceColor);

        // Create and build the route based on the selected route
        meepMeepBot.setRoute(MeepMeepTesting.createRoute(meepMeepBot.getAdapter(), selectedRoute));

        // Run the route on the bot
        meepMeepBot.runAction(meepMeepBot.getRoute().getRouteAction(sideOfField));

        // Add the bot to the MeepMeep field
        meepMeep.addEntity(meepMeepBot.getBot());
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
                return new OBS_Preload_and_One_Specimen(adapter);
            case PRELOAD_AND_ONE_SAMPLE:
                return new NET_Preload_and_One_Sample(adapter);
            case PRELOAD:
                return new Preload(adapter);
            case PRELOAD_AND_TWO_SPECIMENS:
                return new OBS_Preload_and_Two_Specimens(adapter);
            default:
                return new BasicRoute(adapter);
        }
    }
}

