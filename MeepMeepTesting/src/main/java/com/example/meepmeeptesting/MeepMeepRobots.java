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
    public static RoadRunnerBotEntity blueBackstageBotLeft;
    public static RoadRunnerBotEntity blueBackstageBotRight;

    public static RoadRunnerBotEntity redAudienceBot;
    public static RoadRunnerBotEntity redAudienceBotLeft;
    public static RoadRunnerBotEntity redAudienceBotRight;

    public static RoadRunnerBotEntity blueAudienceBot;
    public static RoadRunnerBotEntity blueAudienceBotLeft;
    public static RoadRunnerBotEntity blueAudienceBotRight;

    public static RoadRunnerBotEntity redBackstageBot;
    public static RoadRunnerBotEntity redBackstageBotLeft;
    public static RoadRunnerBotEntity redBackstageBotRight;

    public static RoadRunnerBotEntity roadRunnerBot;

    public static void createRobots( MeepMeep meepMeep ) {

        blueBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(11.5,17.625)
                .build();

        blueBackstageBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(11.5,17.625)
                .build();

        blueBackstageBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(11.5,17.625)
                .build();

        blueAudienceBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(11.5,17.625)
                .build();

        blueAudienceBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .setDimensions(11.5,17.625)
                .build();

        blueAudienceBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .setDimensions(11.5,17.625)
                .build();

        redAudienceBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .setDimensions(11.5,17.625)
                .build();

        redAudienceBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .setDimensions(11.5,17.625)
                .build();

        redAudienceBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .setDimensions(11.5,17.625)
                .build();

        redBackstageBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(11.5,17.625)
                .build();

        redBackstageBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(11.5,17.625)
                .build();

        redBackstageBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(11.5,17.625)
                .build();

        roadRunnerBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .setDimensions(11.5,17.625)
                .build();
    }

    /**
     * METHODS TO SET SIMPLE ROUTES FOR ALL TEAM PROP LOCATIONS
     **/

    public static void setTeamPropCenterRoutes(Routes routes) {
        blueBackstageBot.runAction(routes.getBlueBackstageBotTeamPropCenterRoute());
        blueAudienceBot.runAction(routes.getBlueAudienceBotTeamPropCenterRoute());
        redBackstageBot.runAction(routes.getRedBackstageBotTeamPropCenterRoute());
        redAudienceBot.runAction(routes.getRedAudienceBotTeamPropCenterRoute());
    }

    public static void setTeamPropLeftRoutes(Routes routes) {
        blueBackstageBot.runAction(routes.getBlueBackstageBotTeamPropLeftRoute());
        blueAudienceBot.runAction(routes.getBlueAudienceBotTeamPropLeftRoute());
        redBackstageBot.runAction(routes.getRedBackstageBotTeamPropLeftRoute());
        redAudienceBot.runAction(routes.getRedAudienceBotTeamPropLeftRoute());
    }

    public static void setTeamPropRightRoutes(Routes routes) {
        blueBackstageBot.runAction(routes.getBlueBackstageBotTeamPropRightRoute());
        blueAudienceBot.runAction(routes.getBlueAudienceBotTeamPropRightRoute());
        redBackstageBot.runAction(routes.getRedBackstageBotTeamPropRightRoute());
        redAudienceBot.runAction(routes.getRedAudienceBotTeamPropRightRoute());
    }

    public static void setTeamPropAllRoutes(Routes routes) {
        blueBackstageBot.runAction(routes.getBlueBackstageBotTeamPropCenterRoute());
        blueBackstageBotLeft.runAction(routes.getBlueBackstageBotTeamPropLeftRoute());
        blueBackstageBotRight.runAction(routes.getBlueBackstageBotTeamPropRightRoute());

        blueAudienceBot.runAction(routes.getBlueAudienceBotTeamPropCenterRoute());
        blueAudienceBotLeft.runAction(routes.getBlueAudienceBotTeamPropLeftRoute());
        blueAudienceBotRight.runAction(routes.getBlueAudienceBotTeamPropRightRoute());

        redBackstageBot.runAction(routes.getRedBackstageBotTeamPropCenterRoute());
        redBackstageBotLeft.runAction(routes.getRedBackstageBotTeamPropLeftRoute());
        redBackstageBotRight.runAction(routes.getRedBackstageBotTeamPropRightRoute());

        redAudienceBot.runAction(routes.getRedAudienceBotTeamPropCenterRoute());
        redAudienceBotLeft.runAction(routes.getRedAudienceBotTeamPropLeftRoute());
        redAudienceBotRight.runAction(routes.getRedAudienceBotTeamPropRightRoute());
    }

}
