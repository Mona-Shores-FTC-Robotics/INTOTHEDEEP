package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBot {
    private final RoadRunnerBotEntity bot;
    private final RobotAdapter adapter;
    private Routes route;

    public MeepMeepBot(MeepMeep meepMeep, ColorScheme colorScheme, FieldConstants.AllianceColor allianceColor, FieldConstants.SideOfField sideOfField) {
        // Create the bot
        this.bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(colorScheme)
                .setDimensions(18, 18)
                .setStartPose(FieldConstants.getStartPose(allianceColor, sideOfField))
                .build();

        // Create the adapter with the DriveShim from the bot
        DriveShim driveShim = bot.getDrive();
        this.adapter = new MeepMeepRobotAdapter(driveShim);
    }

    public RoadRunnerBotEntity getBot() {
        return bot;
    }

    public RobotAdapter getAdapter() {
        return adapter;
    }

    public void setRoute(Routes route) {
        this.route = route;
        this.route.BuildRoutes(); // Build the route when it's set
    }

    public Routes getRoute() {
        return this.route;
    }

    public void runAction(Action action) {
        if (action == null) {
            // Log or print an error message if the action is null
            System.out.println("Error: No valid action provided. Cannot run null action.");
            return; // Exit the method without running any action
        }

        // If action is valid, proceed with running it
        bot.runAction(action);
    }
}
