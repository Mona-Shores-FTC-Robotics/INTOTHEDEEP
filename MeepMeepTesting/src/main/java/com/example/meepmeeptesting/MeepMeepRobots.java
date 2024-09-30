package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.Routes;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
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

    public static MeepMeepDriveAdapter blueBackstageAdapter;
    public static MeepMeepDriveAdapter redAudienceAdapter;
    public static MeepMeepDriveAdapter blueAudienceAdapter;
    public static MeepMeepDriveAdapter redBackstageAdapter;

    public static void createRobots( MeepMeep meepMeep ) {

        // Create each robot entity with its own settings
        blueBackstageBot = createBot(meepMeep, new ColorSchemeBlueDark(), FieldConstants.BLUE_BACKSTAGE_START_POSE);
        blueAudienceBot = createBot(meepMeep, new ColorSchemeBlueLight(), FieldConstants.BLUE_AUDIENCE_START_POSE);
        redAudienceBot = createBot(meepMeep, new ColorSchemeRedDark(), FieldConstants.RED_AUDIENCE_START_POSE);
        redBackstageBot = createBot(meepMeep, new ColorSchemeRedLight(), FieldConstants.RED_BACKSTAGE_START_POSE);

        // Set up the drive adapters for each bot
        blueBackstageAdapter = new MeepMeepDriveAdapter(blueBackstageBot.getDrive());
        blueAudienceAdapter = new MeepMeepDriveAdapter(blueAudienceBot.getDrive());
        redAudienceAdapter = new MeepMeepDriveAdapter(redAudienceBot.getDrive());
        redBackstageAdapter = new MeepMeepDriveAdapter(redBackstageBot.getDrive());
    }

    // Helper function to create each robot with specific constraints and colors
    private static RoadRunnerBotEntity createBot(MeepMeep meepMeep, ColorScheme colorScheme, Pose2d startPose) {
        return new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(colorScheme)
                .setDimensions(18, 18)
                .setStartPose(startPose)
                .build();
    }

    public static void setRoutes(Routes routes) {
        redAudienceBot.runAction(routes.getRedAudienceBotRoute());
        blueBackstageBot.runAction(routes.getBlueBackstageBotRoute());
        blueAudienceBot.runAction(routes.getBlueAudienceBotRoute());
        redBackstageBot.runAction(routes.getRedBackstageBotRoute());
    }
}
