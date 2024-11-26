package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;
import com.example.sharedconstants.Routes.Routes;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBot {

    static double MAX_VEL = 60;
    static double MAX_ACCELERATION = 60;
    static double MAX_ANGULAR_VELOCITY = Math.toRadians(360);
    static double MAX_ANGULAR_ACCELERATION = Math.toRadians(360);
    static double TRACK_WIDTH = 13;

    private final RoadRunnerBotEntity bot;
    private final RobotAdapter adapter;
    public static Constraints constraints = new Constraints(MAX_VEL,MAX_ACCELERATION,MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION,TRACK_WIDTH);;
    private Routes route;

    public MeepMeepBot(MeepMeep meepMeep, ColorScheme colorScheme, FieldConstants.AllianceColor allianceColor, FieldConstants.SideOfField sideOfField) {
        constraints = new Constraints(MAX_VEL,MAX_ACCELERATION,MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION,TRACK_WIDTH);;

        bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(constraints.getMaxVel(), constraints.getMaxAccel(), constraints.getMaxAngVel(), constraints.getMaxAngAccel(), constraints.getTrackWidth())
                .setColorScheme(colorScheme)
                .setDimensions(13.5, 16.5)
                .setStartPose(FieldConstants.getStartPose(sideOfField, allianceColor))
                .build();

        // Create the adapter with the DriveShim from the bot
        DriveShim driveShim = bot.getDrive();
        adapter = new MeepMeepRobotAdapter(driveShim);
        adapter.setSideOfField(sideOfField);
        adapter.setAllianceColor(allianceColor); //set alliance color in adapter so it can rotate appropriately
    }

    public RoadRunnerBotEntity getBot() {
        return bot;
    }

    public RobotAdapter getAdapter() {
        return adapter;
    }

    public void setRoute(Routes route) {
        this.route = route;
        route.buildRoute();
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
