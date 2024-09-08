package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.example.sharedconstants.Routes.Routes;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRobots {

    public static RoadRunnerBotEntity blueBackstageBot;
    public static RoadRunnerBotEntity redAudienceBot;
    public static RoadRunnerBotEntity blueAudienceBot;
    public static RoadRunnerBotEntity redBackstageBot;

    public static RoadRunnerBotEntity roadRunnerBot;


    public static void createRobots( MeepMeep meepMeep ) {

        blueBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(18,18)
                .build();

        blueAudienceBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(18,18)
                .build();

        redAudienceBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .setDimensions(18,18)
                .build();

        redBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(18,18)
                .build();
    }

    public static void setRoutes(Routes routes) {
        redAudienceBot.runAction(routes.getRedAudienceBotRoute());
        blueBackstageBot.runAction(routes.getBlueBackstageBotRoute());
        blueAudienceBot.runAction(routes.getBlueAudienceBotRoute());
        redBackstageBot.runAction(routes.getRedBackstageBotRoute());
    }

}
