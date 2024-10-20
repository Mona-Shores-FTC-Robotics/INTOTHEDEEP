package com.example.meepmeeptesting;

import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SideOfField.NET;
import static com.example.sharedconstants.FieldConstants.SideOfField.OBSERVATION;

import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeDarkBlue;
import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeDarkRed;
import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeLightBlue;
import com.example.meepmeeptesting.ColorSchemes.CustomColorSchemeLightRed;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.DoNothing;
import com.example.sharedconstants.Routes.MoveOnly;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_4_Preload_and_3_Samples;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_3_Preload_and_2_Samples;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples;
import com.example.sharedconstants.Routes.NET.NET_Score5_SamplePreload;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_2_Preload_and_1_Sample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_3_Preload_and_2_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_4_Preload_and_3_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike;
import com.example.sharedconstants.Routes.OBS.PushAllAtOnce.OBS_Push2SpikeSamplesInOnePath;
import com.example.sharedconstants.Routes.OBS.PushAllAtOnce.OBS_Push3SpikeSampleInOnePath;
import com.example.sharedconstants.Routes.OBS.SampleFirst.OBS_Score_4_SampleFirst_Push_2_Spike_Samples;
import com.example.sharedconstants.Routes.OBS.SampleFirst.OBS_Score_5_SampleFirst_Push_3_Spike_Samples;
import com.example.sharedconstants.Routes.OBS.SampleFirst.OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Preload;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_2_Preload_and_1_Sample;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    // Set the initial routes to run for each bot
//    private static final RoutesToRun redObservationRoute = RoutesToRun.DO_NOTHING;
//    private static final RoutesToRun blueObservationRoute = redObservationRoute;
//    private static final RoutesToRun redNetRoute = RoutesToRun.NET_SCORE_2_PRELOAD_AND_1_SAMPLE;
//    private static final RoutesToRun blueNetRoute = redNetRoute;

    //If you make the routes null, then it will let you select with numbers
//    private static final RoutesToRun redObservationRoute = null;
//    private static final RoutesToRun blueObservationRoute = null;
//    private static final RoutesToRun redNetRoute = null;
//    private static final RoutesToRun blueNetRoute = null;

    /**
     * Most Reasonable Paths - Alliance Partner does nothing
     * Uncomment one of the sets of routes below to test in MeepMeep.
     */

    // Example: Alliance Partner does nothing, we score 3 specimens (30 POINTS)
    // Uncomment the following lines to test this configuration
//    private static final RoutesToRun redObservationRoute = RoutesToRun.OBS_PUSH_2_SCORE_3_PRELOAD_AND_1_PREMADE_AND_1_SPIKE_SPECIMENS;
//    private static final RoutesToRun blueObservationRoute = redObservationRoute;
//    private static final RoutesToRun redNetRoute = RoutesToRun.DO_NOTHING;
//    private static final RoutesToRun blueNetRoute = redNetRoute;

    // Example: Alliance partner does nothing, we score 1 specimen and 3 samples (34 points)
    // Uncomment the following lines to test this configuration
//    private static final RoutesToRun redObservationRoute = RoutesToRun.DO_NOTHING;
//    private static final RoutesToRun blueObservationRoute = redObservationRoute;
//    private static final RoutesToRun redNetRoute = RoutesToRun.NET_SCORE_4_PRELOAD_AND_3_SAMPLES;
//    private static final RoutesToRun blueNetRoute = RoutesToRun.NET_SCORE_4_PRELOAD_AND_3_SAMPLES_SHORT;

    /**
     * Paths that are Reasonable and Close to Optimized
     * Uncomment these paths to experiment with different autonomous strategies.
     */

    // OBS bot pushes 2 samples and focuses on scoring 4 specimens (Total Auto: 83 - over time)
    // NET bot focuses on scoring 5 samples (only has to grab one from observation since it has a preload)
    // Uncomment the following lines to test this configuration
    private static final RoutesToRun redObservationRoute = RoutesToRun.OBS_PUSH_3_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_SPIKE_SPECIMENS;
    private static final RoutesToRun blueObservationRoute = RoutesToRun.OBS_PUSH_2_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_NEUTRAL_SPECIMENS;
    private static final RoutesToRun redNetRoute = RoutesToRun.NET_SCORE_5_SAMPLE_PRELOAD;
    private static final RoutesToRun blueNetRoute = redNetRoute;

    //  OBS bot scores 4 specimens on high chamber, NET bot scores 5 samples in high basket
    //  OBS bot has a preload specimen and NET bot has a preload sample
    //  OBS bot:
    //      - Scores preload specimen on chamber
    //      - Pushes spike sample 1 to human player
    //      - Grabs premade specimen from human player and scores it on chamber
    //      - Pushes spike sample 2 to human player
    //      - Grabs specimen from human player (spike sample 1 turned into specimen) and scores it on chamber
    //      - Grabs specimen from human player (spike sample 2 turned into specimen) and scores it on chamber
    //  NET bot:
    //      - Scores the preload sample
    //      - Grabs the three neutrals and scores them
    //      - Grabs second yellow sample from Human Player

    /**
     * Paired Paths - these are designed to work together, but need work
     * Uncomment these paths to explore paired strategies.
     */

    // OBS bot pushes spike samples in one path
    // NET bot does its thing and tacks on two samples from human player at end (Total Auto: 90 - close)
    // Uncomment the following lines to test this scenario
//    private static final RoutesToRun redObservationRoute = RoutesToRun.OBS_PUSH_3_SPIKE_SAMPLES_IN_ONE_PATH;
//    private static final RoutesToRun blueObservationRoute = redObservationRoute;
//    private static final RoutesToRun redNetRoute = RoutesToRun.NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE_SHORT;
//    private static final RoutesToRun blueNetRoute = redNetRoute;

    // Strategy:
    //  Both robots start with a preload specimen, which means the human player has no specimens to make at the beginning of auto
    //  OBS bot scores the preload and then pushes three samples into observation zone to give the human player time to make specimens
    //  NET bot scores the preload and the three neutral spike samples, plenty of time left to grab two yellow samples from Observation Zone
    //  OBS bot has to stay out of the way of the NET bot.
    //  OBS bot needs to be pretty fast to pull this off, might have to score one fewer, but general logic should still work

    // **Obs Bot Sample First**
    // OBS bot starts with sample first to allow the human player to prepare specimens (Total Auto: 80)
    // Uncomment the following lines to test this scenario

//    private static final RoutesToRun redObservationRoute = RoutesToRun.OBS_SCORE_4_SAMPLEFIRST_PUSH_2_SPIKE_SAMPLES;
//    private static final RoutesToRun blueObservationRoute = redObservationRoute;
//    private static final RoutesToRun redNetRoute = RoutesToRun.NET_SCORE_5_PRELOAD_AND_3_SAMPLES_AND_1_HUMAN_PLAYER_SAMPLE;
//    private static final RoutesToRun blueNetRoute = redNetRoute;

    // Strategy:
    //  4 specimens, 5 samples
    //  OBS bot:
    //      - Scores preload sample
    //      - Pushes three samples into observation zone, grabbing a specimen each time to score
    //  NET bot:
    //      - Scores the preload specimen and the three neutral spike samples
    //      - Grabs two yellow samples from Observation Zone

    enum RoutesToRun {
        NET_SCORE_1_PRELOAD,
        OBS_SCORE_1_PRELOAD,
        MOVE_ONLY,

        //Pickup the samples on their long side
        NET_SCORE_2_PRELOAD_AND_1_SAMPLE,
        NET_SCORE_3_PRELOAD_AND_2_SAMPLES,
        NET_SCORE_4_PRELOAD_AND_3_SAMPLES,
        NET_SCORE_5_PRELOAD_AND_3_SAMPLES_AND_1_HUMAN_PLAYER_SAMPLE,
        NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE,

        //Pickup the samples on their short side
        NET_SCORE_2_PRELOAD_AND_1_SAMPLE_SHORT,
        NET_SCORE_3_PRELOAD_AND_2_SAMPLES_SHORT,
        NET_SCORE_4_PRELOAD_AND_3_SAMPLES_SHORT,
        NET_SCORE_5_PRELOAD_AND_3_SAMPLES_AND_1_HUMAN_PLAYER_SAMPLE_SHORT,
        NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE_SHORT,

        //Path to push first and second neutral samples
        OBS_PUSH_2_SCORE_3_PRELOAD_AND_1_PREMADE_AND_1_SPIKE_SPECIMENS,

        //Path to push the third neutral sample - Assuming partner has no preload (do nothing or just moves)
        OBS_PUSH_3_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_SPIKE_SPECIMENS,
        OBS_PUSH_3_SCORE_5_PRELOAD_AND_1_PREMADE_AND_3_SPIKE_SPECIMENS,

        //Save time by not pushing the third neutral sample - Assuming partner has no preload (do nothing or just moves)
        OBS_PUSH_2_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_NEUTRAL_SPECIMENS,
        OBS_SCORE_1_SAMPLE_PRELOAD_PUSH_1_SPIKE_SCORE_1_PREMADE,

        //Experiment with double specimen preload - push multiple samples in one path to give human player time to make specimen
        OBS_PUSH_3_SPIKE_SAMPLES_IN_ONE_PATH,
        OBS_PUSH_2_SPIKE_SAMPLES_IN_ONE_PATH,

        //Experiment with obs bot doing a sample first so human player can get a specimen ready
        OBS_SCORE_4_SAMPLEFIRST_PUSH_2_SPIKE_SAMPLES,
        DO_NOTHING, NET_SCORE_5_SAMPLE_PRELOAD, OBS_SCORE_4_SAMPLEFIRST_PUSH_3_SPIKE_SAMPLES

    }

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

        final RoutesToRun[] observationRoute = new RoutesToRun[1];
        final RoutesToRun[] netRoute = new RoutesToRun[1];

        if (redObservationRoute != null && blueObservationRoute != null && redNetRoute != null && blueNetRoute != null) {
            observationRoute[0] = redObservationRoute;
            netRoute[0] = redNetRoute;
            System.out.println("Using predefined routes:");
            System.out.println("Red Observation Bot: " + redObservationRoute);
            System.out.println("Blue Observation Bot: " + blueObservationRoute);
            System.out.println("Red NET Bot: " + redNetRoute);
            System.out.println("Blue NET Bot: " + blueNetRoute);
        } else if (!useRandomSelection) {
            Scanner scanner = new Scanner(System.in);
            observationRoute[0] = selectRoute(scanner, "Observation Bot", "OBS");
            netRoute[0] = selectRoute(scanner, "NET Bot", "NET");
            System.out.println("Manually selected routes:");
            System.out.println("\u001B[36mObservation Bot: " + observationRoute[0] + "\u001B[0m");
            System.out.println("\u001B[33mNET Bot: " + netRoute[0] + "\u001B[0m");
        } else {
            observationRoute[0] = getRandomRoute("OBS");
            netRoute[0] = getRandomRoute("NET");
            System.out.println("Randomly selected routes:");
            System.out.println("Observation Bot: " + observationRoute[0]);
            System.out.println("NET Bot: " + netRoute[0]);
        }

        List<MeepMeepBot> entities = new ArrayList<>();


        // Create the robots and configure them initially
        entities.add(createAdaptedBotAndRunRoute(meepMeep, RED, NET, new CustomColorSchemeDarkRed(), netRoute[0]));
        entities.add(createAdaptedBotAndRunRoute(meepMeep, RED, OBSERVATION, new CustomColorSchemeLightRed(), observationRoute[0]));
        entities.add(createAdaptedBotAndRunRoute(meepMeep, BLUE, NET, new CustomColorSchemeDarkBlue(), netRoute[0]));
        entities.add(createAdaptedBotAndRunRoute(meepMeep, BLUE, OBSERVATION, new CustomColorSchemeLightBlue(), observationRoute[0]));
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

    private static RoutesToRun getRandomRoute(String prefix) {
        RoutesToRun[] routes = RoutesToRun.values();
        Random random = new Random();
        RoutesToRun selectedRoute;

        do {
            selectedRoute = routes[random.nextInt(routes.length)];
        } while (!selectedRoute.name().startsWith(prefix));

        return selectedRoute;
    }

    private static MeepMeepBot createAdaptedBotAndRunRoute(MeepMeep meepMeep,
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

        return meepMeepBot;
    }

    // Helper method to create the route based on the route selection
    private static Routes createRoute(RobotAdapter adapter, RoutesToRun routeToRunSelection) {
        switch (routeToRunSelection) {
            case NET_SCORE_2_PRELOAD_AND_1_SAMPLE:
                return new NET_Score_2_Preload_and_1_Sample(adapter);
            case NET_SCORE_3_PRELOAD_AND_2_SAMPLES:
                return new NET_Score_3_Preload_and_2_Samples(adapter);
            case NET_SCORE_4_PRELOAD_AND_3_SAMPLES:
                return new NET_Score_4_Preload_and_3_Samples(adapter);
            case NET_SCORE_5_PRELOAD_AND_3_SAMPLES_AND_1_HUMAN_PLAYER_SAMPLE:
                return new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample(adapter);
            case NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE:
                return new NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples(adapter);
            case NET_SCORE_2_PRELOAD_AND_1_SAMPLE_SHORT:
                return new NET_Score_2_Preload_and_1_Sample_Short(adapter);
            case NET_SCORE_3_PRELOAD_AND_2_SAMPLES_SHORT:
                return new NET_Score_3_Preload_and_2_Samples_Short(adapter);
            case NET_SCORE_4_PRELOAD_AND_3_SAMPLES_SHORT:
                return new NET_Score_4_Preload_and_3_Samples_Short(adapter);
            case NET_SCORE_5_PRELOAD_AND_3_SAMPLES_AND_1_HUMAN_PLAYER_SAMPLE_SHORT:
                return new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(adapter);
            case NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE_SHORT:
                return new NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short(adapter);
            case OBS_PUSH_2_SCORE_3_PRELOAD_AND_1_PREMADE_AND_1_SPIKE_SPECIMENS:
                return new OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike(adapter);
            case OBS_PUSH_3_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_SPIKE_SPECIMENS:
                return new OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
            case OBS_PUSH_3_SCORE_5_PRELOAD_AND_1_PREMADE_AND_3_SPIKE_SPECIMENS:
                return new OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike(adapter);
            case OBS_PUSH_2_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_NEUTRAL_SPECIMENS:
                return new OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
            case OBS_SCORE_1_SAMPLE_PRELOAD_PUSH_1_SPIKE_SCORE_1_PREMADE:
                return new OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade(adapter);
            case NET_SCORE_1_PRELOAD:
                return new NET_Score_1_Preload(adapter);
            case OBS_SCORE_1_PRELOAD:
                return new OBS_Score_1_Specimen_Preload(adapter);
            case OBS_SCORE_4_SAMPLEFIRST_PUSH_2_SPIKE_SAMPLES:
                return new OBS_Score_4_SampleFirst_Push_2_Spike_Samples(adapter);
            case OBS_SCORE_4_SAMPLEFIRST_PUSH_3_SPIKE_SAMPLES:
                return new OBS_Score_5_SampleFirst_Push_3_Spike_Samples(adapter);
            case OBS_PUSH_3_SPIKE_SAMPLES_IN_ONE_PATH:
                return new OBS_Push3SpikeSampleInOnePath(adapter);
            case OBS_PUSH_2_SPIKE_SAMPLES_IN_ONE_PATH:
                return new OBS_Push2SpikeSamplesInOnePath(adapter);
            case NET_SCORE_5_SAMPLE_PRELOAD:
                return new NET_Score5_SamplePreload(adapter);
            case MOVE_ONLY:
                return new MoveOnly(adapter);
            case DO_NOTHING:
            default:
                return new DoNothing(adapter);
        }
    }
}


