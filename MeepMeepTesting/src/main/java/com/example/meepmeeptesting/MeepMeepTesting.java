package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.NET_PRELOAD;
import static com.example.meepmeeptesting.MeepMeepTesting.RoutesToRun.OBS_PRELOAD_AND_TWO_SPECIMENS;
import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SideOfField.NET;
import static com.example.sharedconstants.FieldConstants.SideOfField.OBSERVATION;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.MoveOnly;
import com.example.sharedconstants.Routes.NET.NET_Net_Preload_and_Three_Samples;
import com.example.sharedconstants.Routes.NET.NET_Net_Preload_and_Two_Samples;
import com.example.sharedconstants.Routes.OBS.ObservationPreload;
import com.example.sharedconstants.Routes.OBS.Push.OBS__Score__Preload__PushTwo_TwoNeutralSpecimens_and_Score_Four_Premade_Specimens;
import com.example.sharedconstants.Routes.OBS.Push.OBS__Score__Preload__PushTwo_TwoNeutralSpecimens_and_Score_Three_Premade_Specimens;
import com.example.sharedconstants.Routes.OBS.Push.OBS___Score___Push___Five_NeutralScoreFour_Specimens;
import com.example.sharedconstants.Routes.OBS.Push.OBS___Score___Push___Four_NeutralScoreFour_Specimens;
import com.example.sharedconstants.Routes.OBS.OBS_Score_Preload_and_One_Premade_Specimen;
import com.example.sharedconstants.Routes.OBS.OBS_Score_Preload_and_Two_Premade_Specimens;
import com.example.sharedconstants.Routes.OBS.OBS_Score_Preload_Push_One_Neutral_Score_One_Premade_Specimen;
import com.example.sharedconstants.Routes.OBS.OBS_Score_Preload_Push_Two_Neutral_Specimens_and_Score_Two_Premade_Specimens;
import com.example.sharedconstants.Routes.NET.NetPreload;
import com.example.sharedconstants.Routes.NET.NET_Net_Preload_and_One_Sample;
import com.example.sharedconstants.Routes.Routes;
import com.example.sharedconstants.Routes.OBS.Push.OBS_Score_Push_Three_Neutral_Score_Four_Specimens;
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

    private static final RoutesToRun redObservationRoute = OBS_PRELOAD_AND_TWO_SPECIMENS ;
    private static final RoutesToRun blueObservationRoute = OBS_PRELOAD_AND_TWO_SPECIMENS;
    private static final RoutesToRun redNetRoute = NET_PRELOAD;
    private static final RoutesToRun blueNetRoute = NET_PRELOAD;

    enum RoutesToRun {
        NET_PRELOAD,
        OBS_PRELOAD,
        MOVE_ONLY,
        NET_PRELOAD_AND_ONE_SAMPLE,
        NET_PRELOAD_AND_TWO_SAMPLES,
        NET_PRELOAD_AND_THREE_SAMPLES,
        OBS_PRELOAD_AND_ONE_SPECIMEN,
        OBS_PRELOAD_AND_TWO_SPECIMENS,
        OBS_PRELOAD_PUSH_AND_ONE_SPECIMEN,
        OBS_PRELOAD_PUSH_AND_TWO_SPECIMENS,
        OBS_PRELOAD_PUSH_AND_THREE_SPECIMENS,
        OBS_PRELOAD_PUSH_AND_FOUR_SPECIMENS,
        OBS_PRELOAD_PUSH_AND_FIVE_SPECIMENS,
        OBS_PRELOAD_PUSH_TWO_AND_THREE_SPECIMENS,
        OBS_PRELOAD_PUSH_TWO_AND_FOUR_SPECIMENS
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
            case NET_PRELOAD_AND_ONE_SAMPLE:
                return new NET_Net_Preload_and_One_Sample(adapter);
            case NET_PRELOAD_AND_TWO_SAMPLES:
                return new NET_Net_Preload_and_Two_Samples(adapter);
            case NET_PRELOAD_AND_THREE_SAMPLES:
                return new NET_Net_Preload_and_Three_Samples(adapter);
            case OBS_PRELOAD_AND_ONE_SPECIMEN:
                return new OBS_Score_Preload_and_One_Premade_Specimen(adapter);
            case OBS_PRELOAD_AND_TWO_SPECIMENS:
                return new OBS_Score_Preload_and_Two_Premade_Specimens(adapter);
            case OBS_PRELOAD_PUSH_AND_ONE_SPECIMEN:
                return new OBS_Score_Preload_Push_One_Neutral_Score_One_Premade_Specimen(adapter);
            case OBS_PRELOAD_PUSH_AND_TWO_SPECIMENS:
                return new OBS_Score_Preload_Push_Two_Neutral_Specimens_and_Score_Two_Premade_Specimens(adapter);
            case OBS_PRELOAD_PUSH_AND_THREE_SPECIMENS:
                return new OBS_Score_Push_Three_Neutral_Score_Four_Specimens(adapter);
            case OBS_PRELOAD_PUSH_AND_FOUR_SPECIMENS:
                return new OBS___Score___Push___Four_NeutralScoreFour_Specimens(adapter);
            case OBS_PRELOAD_PUSH_AND_FIVE_SPECIMENS:
                return new OBS___Score___Push___Five_NeutralScoreFour_Specimens(adapter);
            case OBS_PRELOAD_PUSH_TWO_AND_THREE_SPECIMENS:
                return new OBS__Score__Preload__PushTwo_TwoNeutralSpecimens_and_Score_Three_Premade_Specimens(adapter);
            case OBS_PRELOAD_PUSH_TWO_AND_FOUR_SPECIMENS:
                return new OBS__Score__Preload__PushTwo_TwoNeutralSpecimens_and_Score_Four_Premade_Specimens(adapter);
            case NET_PRELOAD:
                return new NetPreload(adapter);
            case OBS_PRELOAD:
                return new ObservationPreload(adapter);
            case MOVE_ONLY:
            default:
                return new MoveOnly(adapter);
        }
    }
}

