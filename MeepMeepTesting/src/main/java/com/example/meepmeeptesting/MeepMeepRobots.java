package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
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

    public static void setTeamPropCenterRoutes(Action blueBackstageRoute, Action blueAudienceRoute, Action redBackstageRoute, Action redAudienceRoute) {
        blueBackstageBot.runAction(blueBackstageRoute);
        blueAudienceBot.runAction(blueAudienceRoute);
        redBackstageBot.runAction(redBackstageRoute);
        redAudienceBot.runAction(redAudienceRoute);
    }

    public static void setTeamPropLeftRoutes(Action blueBackstageTeamPropLeftRoute, Action blueAudienceTeamPropLeftRoute, Action redBackstageTeamPropLeftRoute, Action redAudienceTeamPropLeftRoute) {
        blueBackstageBot.runAction(blueBackstageTeamPropLeftRoute);
        blueAudienceBot.runAction(blueAudienceTeamPropLeftRoute);
        redBackstageBot.runAction(redBackstageTeamPropLeftRoute);
        redAudienceBot.runAction(redAudienceTeamPropLeftRoute);
    }

    public static void setTeamPropRightRoutes(Action blueBackstageRoute, Action blueAudienceRoute, Action redBackstageRoute, Action redAudienceRoute) {
        blueBackstageBot.runAction(blueBackstageRoute);
        blueAudienceBot.runAction(blueAudienceRoute);
        redBackstageBot.runAction(redBackstageRoute);
        redAudienceBot.runAction(redAudienceRoute);
    }

    public static void setTeamPropAllRoutes(
            Action blueBackstageBotTeamPropLeftRoute, Action blueBackstageBotTeamPropCenterRoute, Action blueBackstageBotTeamPropRightRoute,
            Action blueAudienceBotTeamPropCenterRoute, Action blueAudienceBotTeamPropLeftRoute, Action blueAudienceBotTeamPropRightSequentialAction,
            Action redBackstageBotTeamPropCenterRoute, Action redBackstageBotTeamPropLeftRoute, Action redBackstageBotTeamPropRightRoute,
            Action redAudienceBotTeamPropCenterRoute, Action redAudienceBotTeamPropLeftRoute, Action redAudienceBotTeamPropRightRoute)
        {
        blueBackstageBot.runAction(blueBackstageBotTeamPropCenterRoute);
        blueBackstageBotLeft.runAction(blueBackstageBotTeamPropLeftRoute);
        blueBackstageBotRight.runAction(blueBackstageBotTeamPropRightRoute);

        blueAudienceBot.runAction(blueAudienceBotTeamPropCenterRoute);
        blueAudienceBotLeft.runAction(blueAudienceBotTeamPropLeftRoute);
        blueAudienceBotRight.runAction(blueAudienceBotTeamPropRightSequentialAction);

        redBackstageBot.runAction(redBackstageBotTeamPropCenterRoute);
        redBackstageBotLeft.runAction(redBackstageBotTeamPropLeftRoute);
        redBackstageBotRight.runAction(redBackstageBotTeamPropRightRoute);

        redAudienceBot.runAction(redAudienceBotTeamPropCenterRoute);
        redAudienceBotLeft.runAction(redAudienceBotTeamPropLeftRoute);
        redAudienceBotRight.runAction(redAudienceBotTeamPropRightRoute);
    }

}
