package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE_SHORT;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.OBS_PUSH_3_SCORE_5_PRELOAD_AND_1_PREMADE_AND_3_SPIKE_SPECIMENS;
import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SideOfField.NET;
import static com.example.sharedconstants.FieldConstants.SideOfField.OBSERVATION;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.MoveOnly;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_4_Preload_and_3_Samples;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_3_Preload_and_2_Samples;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_2_Preload_and_1_Sample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_3_Preload_and_2_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_4_Preload_and_3_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short;
import com.example.sharedconstants.Routes.OBS.OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.Push2Alt.OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike;
import com.example.sharedconstants.Routes.OBS.OBS_Push_1_Score_2_Specimens_Preload_And_1_Premade;
import com.example.sharedconstants.Routes.OBS.OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Preload;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_2_Preload_and_1_Sample;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.OBS.OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike;
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

    private static final RoutesToRun redObservationRoute = OBS_PUSH_3_SCORE_5_PRELOAD_AND_1_PREMADE_AND_3_SPIKE_SPECIMENS;
    private static final RoutesToRun blueObservationRoute = redObservationRoute;
    private static final RoutesToRun redNetRoute = NET_SCORE_6_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE_SHORT;
    private static final RoutesToRun blueNetRoute = redNetRoute;

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
        OBS_PUSH_1_SCORE_2_PRELOAD_AND_1_PREMADE_SPECIMEN,
        OBS_PUSH_2_SCORE_3_PRELOAD_AND_1_PREMADE_AND_1_SPIKE_SPECIMENS,

        //Path to push the third neutral sample
        OBS_PUSH_3_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_SPIKE_SPECIMENS,
        OBS_PUSH_3_SCORE_5_PRELOAD_AND_1_PREMADE_AND_3_SPIKE_SPECIMENS,

        //Save time by not pushing the third neutral sample
        OBS_PUSH_2_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_NEUTRAL_SPECIMENS,
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Create the robots and configure them
        createAdaptedBotAndRunRoute(meepMeep, RED, NET, new ColorSchemeRedDark(), redNetRoute);
        createAdaptedBotAndRunRoute(meepMeep, RED, OBSERVATION, new ColorSchemeRedLight(), redObservationRoute);
        createAdaptedBotAndRunRoute(meepMeep, BLUE, NET, new ColorSchemeBlueDark(), blueNetRoute);
        createAdaptedBotAndRunRoute(meepMeep, BLUE, OBSERVATION, new ColorSchemeBlueLight(), blueObservationRoute);
        startMeepMeep(meepMeep);
    }
    // Create a robot and associated adapter and run the selected route
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
            case OBS_PUSH_1_SCORE_2_PRELOAD_AND_1_PREMADE_SPECIMEN:
                return new OBS_Push_1_Score_2_Specimens_Preload_And_1_Premade(adapter);
            case OBS_PUSH_2_SCORE_3_PRELOAD_AND_1_PREMADE_AND_1_SPIKE_SPECIMENS:
                return new OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike(adapter);
            case OBS_PUSH_3_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_SPIKE_SPECIMENS:
                return new OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
            case OBS_PUSH_3_SCORE_5_PRELOAD_AND_1_PREMADE_AND_3_SPIKE_SPECIMENS:
                return new OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike(adapter);
            case OBS_PUSH_2_SCORE_4_PRELOAD_AND_1_PREMADE_AND_2_NEUTRAL_SPECIMENS:
                return new OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
            case NET_SCORE_1_PRELOAD:
                return new NET_Score_1_Preload(adapter);
            case OBS_SCORE_1_PRELOAD:
                return new OBS_Score_1_Specimen_Preload(adapter);
            case MOVE_ONLY:
            default:
                return new MoveOnly(adapter);
        }
    }
}

