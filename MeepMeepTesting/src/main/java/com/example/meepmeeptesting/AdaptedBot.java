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

public class AdaptedBot {
    private final RoadRunnerBotEntity bot;
    private final RobotAdapter adapter;
    private Routes route;


    public AdaptedBot(MeepMeep meepMeep, ColorScheme colorScheme, Pose2d startPose) {
        // Create the bot
        this.bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(colorScheme)
                .setDimensions(18, 18)
                .setStartPose(startPose)
                .build();

        // Create the adapter with the DriveShim from the bot
        DriveShim driveShim = bot.getDrive();
        this.adapter = new MeepMeepDriveAdapter(driveShim);
    }

    public RoadRunnerBotEntity getBot() {
        return bot;
    }

    public RobotAdapter getAdapter() {
        return adapter;
    }

    public void setAllianceColor(FieldConstants.AllianceColor allianceColor) {
        adapter.setAllianceColor(allianceColor);
    }

    public void setSideOfField(FieldConstants.SideOfField sideOfField) {
        adapter.setSideOfField(sideOfField);
    }

    public void setRoute(Routes route) {
        this.route = route;
        this.route.BuildRoutes(); // Build the route when it's set
    }

    public Routes getRoute() {
        return this.route;
    }

    public void runAction(Action action) {
        bot.runAction(action);
    }
}
