package com.example.meepmeeptesting;

import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SideOfField.NET;
import static com.example.sharedconstants.FieldConstants.SideOfField.OBSERVATION;
import static com.example.sharedconstants.RoutesToRun.DO_NOTHING;
import static com.example.sharedconstants.RoutesToRun.NET_SCORE_1_SAMPLE_PRELOAD;
import static com.example.sharedconstants.RoutesToRun.OBS_SCORE_4_FRUITPORT;
import static com.example.sharedconstants.RoutesToRun.OBS_SCORE_4_FRUITPORT_IMPROVED;
import static com.example.sharedconstants.RoutesToRun.OBS_SCORE_4_PICKUP_AT_CORNER;
import static com.example.sharedconstants.RoutesToRun.OBS_SCORE_4_PICKUP_AT_TILE_SEAM;
import static com.example.sharedconstants.RoutesToRun.OBS_SCORE_4_PICKUP_AT_TRIAGNLE_TIP;

import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeDarkBlue;
import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeDarkRed;
import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeLightBlue;
import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeLightRed;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.NET.SamplePreload.NET_Score_1_Sample_Preload;
import com.example.sharedconstants.Routes.NET.SamplePreload.NET_Score_2_Sample_Preload;
import com.example.sharedconstants.Routes.NET.SamplePreload.NET_Score_3_Sample_Preload;
import com.example.sharedconstants.Routes.NET.SamplePreload.NET_Score_4_Sample_Preload;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_2_Preload_and_1_Sample;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_3_Preload_and_2_Samples;
import com.example.sharedconstants.Routes.OBS.OBS_Score4_Fruitport_Improved;
import com.example.sharedconstants.Routes.OBS.OBS_Score4_PickupAtTriangleTip;
import com.example.sharedconstants.Routes.OBS.Old.OBS_SQUARE_AUTO;
import com.example.sharedconstants.Routes.OBS.OBS_Score4_PickupAtCorner;
import com.example.sharedconstants.Routes.OBS.OBS_Score4_PickupAtTileSeam;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score4_NO_PRELOAD_SPECTACULAR;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score4_Preload_Push_All_And_Pickup_At_Triangle;
import com.example.sharedconstants.Routes.OBS.OBS_Score4_Fruitport;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score5_Preload_Ground_Pickup_And_Dump_And_Pickup_At_Triangle;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.DoNothing;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score5_SamplePreload;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_4_Preload_and_3_Samples;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score4_Preload_Push_Two_And_Pickup_At_Triangle;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score5_Leave_Preload_Push_All_And_Pickup_At_Triangle;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score5_Preload_Push_All_And_Pickup_At_Triangle;
import com.example.sharedconstants.Routes.OBS.Old.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.RoutesToRun;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    private static final RoutesToRun redObservationRoute = OBS_SCORE_4_PICKUP_AT_CORNER;
    private static final RoutesToRun blueObservationRoute = OBS_SCORE_4_FRUITPORT_IMPROVED;
    private static final RoutesToRun redNetRoute = NET_SCORE_1_SAMPLE_PRELOAD;
    private static final RoutesToRun blueNetRoute = redNetRoute;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        String filePath = "intothedeep2.png";  // Customize the field background
        try {
            Image img = ImageIO.read(new File(filePath));
            meepMeep.setBackground(img);
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setDarkMode(false)
                .setBackgroundAlpha(.95f);

        boolean useRandomSelection = false; // Hardcoded toggle for random selection

        final RoutesToRun[] observationRoute = new RoutesToRun[2];
        final RoutesToRun[] netRoute = new RoutesToRun[2];

        if (redObservationRoute != null && blueObservationRoute != null && redNetRoute != null && blueNetRoute != null) {
            observationRoute[0] = redObservationRoute;
            netRoute[0] = redNetRoute;

            observationRoute[1] = blueObservationRoute; // Ensure blue observation route is used
            netRoute[1] = blueNetRoute;                // Ensure blue net route is used

            System.out.println("Using predefined routes:");
            System.out.println("Red Observation Bot: " + redObservationRoute);
            System.out.println("Blue Observation Bot: " + blueObservationRoute);
            System.out.println("Red NET Bot: " + redNetRoute);
            System.out.println("Blue NET Bot: " + blueNetRoute);
            createAdaptedBotAndRunRoute(meepMeep, RED, NET, new CustomColorSchemeDarkRed(), netRoute[0]);
            createAdaptedBotAndRunRoute(meepMeep, BLUE, NET, new CustomColorSchemeDarkBlue(), netRoute[1]);
            createAdaptedBotAndRunRoute(meepMeep, RED, OBSERVATION, new CustomColorSchemeLightRed(), observationRoute[0]);
            createAdaptedBotAndRunRoute(meepMeep, BLUE, OBSERVATION, new CustomColorSchemeLightBlue(), observationRoute[1]);
        } else if (!useRandomSelection) {
            Scanner scanner = new Scanner(System.in);
            observationRoute[0] = selectRoute(scanner, "Observation Bot", "OBS");
            netRoute[0] = selectRoute(scanner, "NET Bot", "NET");
            System.out.println("Manually selected routes:");
            System.out.println("\u001B[36mObservation Bot: " + observationRoute[0] + "\u001B[0m");
            System.out.println("\u001B[33mNET Bot: " + netRoute[0] + "\u001B[0m");
            createAdaptedBotAndRunRoute(meepMeep, RED, NET, new CustomColorSchemeDarkRed(), netRoute[0]);
            createAdaptedBotAndRunRoute(meepMeep, BLUE, NET, new CustomColorSchemeDarkBlue(), netRoute[0]);
            createAdaptedBotAndRunRoute(meepMeep, RED, OBSERVATION, new CustomColorSchemeLightRed(), observationRoute[0]);
            createAdaptedBotAndRunRoute(meepMeep, BLUE, OBSERVATION, new CustomColorSchemeLightBlue(), observationRoute[0]);
        }


        // Start MeepMeep
        meepMeep.start();
    }

    private static RoutesToRun selectRoute(Scanner scanner, String botDescription, String prefix) {
        System.out.println("Select route for " + botDescription + ":");
        RoutesToRun[] routes = RoutesToRun.values();

        // Display available routes that match the given prefix, renumbered sequentially
        int displayIndex = 0;
        for (int i = 0; i < routes.length; i++) {
            if (routes[i].name().startsWith(prefix)) {
                System.out.println(displayIndex + ": " + routes[i]);
                displayIndex++;
            }
        }

        // Get user's selection
        int routeIndex = -1;
        while (routeIndex < 0 || routeIndex >= displayIndex) {
            System.out.print("Enter a valid number for " + botDescription + " route: ");
            if (scanner.hasNextInt()) {
                routeIndex = scanner.nextInt();
                if (routeIndex < 0 || routeIndex >= displayIndex) {
                    System.out.println("Invalid selection. Please try again.");
                }
            } else {
                System.out.println("Invalid input. Please enter a number.");
                scanner.next(); // Clear invalid input
            }
        }

        // Retrieve the corresponding route based on the sequential index
        displayIndex = 0;
        for (int i = 0; i < routes.length; i++) {
            if (routes[i].name().startsWith(prefix)) {
                if (displayIndex == routeIndex) {
                    return routes[i];
                }
                displayIndex++;
            }
        }

        return null; // This should never be reached
    }

    private static void createAdaptedBotAndRunRoute(MeepMeep meepMeep,
                                                    FieldConstants.AllianceColor allianceColor,
                                                    FieldConstants.SideOfField sideOfField,
                                                    ColorScheme colorScheme,
                                                    RoutesToRun selectedRoute) {
        // Create the adapted bot
        MeepMeepBot meepMeepBot = new MeepMeepBot(
                meepMeep, colorScheme, allianceColor, sideOfField); // Default start

        // Create and build the route based on the selected route
        meepMeepBot.setRoute(MeepMeepTesting.createRoute(meepMeepBot.getAdapter(), selectedRoute));

        // Run the route on the bot
        meepMeepBot.runAction(meepMeepBot.getRoute().getRouteAction(sideOfField));

        // Add the bot to the MeepMeep field
        meepMeep.addEntity(meepMeepBot.getBot());

    }

    // Helper method to create the route based on the route selection
    private static Routes createRoute(RobotAdapter adapter, RoutesToRun routeToRunSelection) {
        switch (routeToRunSelection) {

            case NET_SCORE_2_SPECIMEN_PRELOAD_AND_1_SAMPLE:
                return new NET_Score_2_Preload_and_1_Sample(adapter);

            case NET_SCORE_3_SPECIMEN_PRELOAD_AND_2_SAMPLES:
                return new NET_Score_3_Preload_and_2_Samples(adapter);

            case NET_SCORE_4_SPECIMEN_PRELOAD_AND_3_SAMPLES:
                return new NET_Score_4_Preload_and_3_Samples(adapter);

            case NET_SCORE_5_SAMPLE_PRELOAD:
                return new NET_Score5_SamplePreload(adapter);

            case OBS_SCORE_4_FRUITPORT:
                return new OBS_Score4_Fruitport(adapter);

            case OBS_SCORE_4_NO_PRELOAD:
                return new OBS_Score4_NO_PRELOAD_SPECTACULAR(adapter);

            case OBS_SCORE_4_PICKUP_AT_TILE_SEAM:
                return new OBS_Score4_PickupAtTileSeam(adapter);

            case OBS_SCORE_4_PICKUP_AT_CORNER:
                return new OBS_Score4_PickupAtCorner(adapter);

            case OBS_SCORE_4_FRUITPORT_IMPROVED:
                return new OBS_Score4_Fruitport_Improved(adapter);

            case OBS_SCORE_4_PICKUP_AT_TRIAGNLE_TIP:
                return new OBS_Score4_PickupAtTriangleTip(adapter);

            case OBS_SCORE_4_PRELOAD_PUSH_ALL_AND_PICKUP_AT_TRIANGLE:
                return new OBS_Score4_Preload_Push_All_And_Pickup_At_Triangle(adapter);

            case OBS_SCORE_4_PRELOAD_PUSH_TWO_AND_PICKUP_AT_TRIANGLE:
                return new OBS_Score4_Preload_Push_Two_And_Pickup_At_Triangle(adapter);

            case OBS_SCORE_5_PRELOAD_PUSH_ALL_AND_PICKUP_AT_TRIANGLE:
                return new OBS_Score5_Preload_Push_All_And_Pickup_At_Triangle(adapter);

            case OBS_SCORE_5_LEAVE_PRELOAD_PUSH_AND_PICKUP_AT_TRIANGLE:
                return new OBS_Score5_Leave_Preload_Push_All_And_Pickup_At_Triangle(adapter);

            case OBS_SCORE_5_PRELOAD_GROUND_PICKUP_AND_DUMP_AND_PICKUP_AT_TRIANGLE:
                return new OBS_Score5_Preload_Ground_Pickup_And_Dump_And_Pickup_At_Triangle(adapter);

            case NET_SCORE_1_SPECIMEN_PRELOAD:
                return new NET_Score_1_Specimen_Preload(adapter);

            case OBS_SCORE_1_PRELOAD:
                return new OBS_Score_1_Specimen_Preload(adapter);
            case NET_SCORE_1_SAMPLE_PRELOAD:
                return new NET_Score_1_Sample_Preload(adapter);
            case NET_SCORE_2_SAMPLE_PRELOAD:
                return new NET_Score_2_Sample_Preload(adapter);
            case NET_SCORE_3_SAMPLE_PRELOAD:
                return new NET_Score_3_Sample_Preload(adapter);
            case NET_SCORE_4_SAMPLE_PRELOAD:
                return new NET_Score_4_Sample_Preload(adapter);

            case OBS_SQUARE_AUTO:
                return new OBS_SQUARE_AUTO(adapter);

            case DO_NOTHING:
            default:
                return new DoNothing(adapter);
        }
    }
}


