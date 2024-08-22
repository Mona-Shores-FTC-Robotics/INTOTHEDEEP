package com.example.meepmeeptesting;

import static com.example.sharedconstants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.BLUE_BACKSTAGE_START_POSE;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.FieldConstants;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(1000);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), FieldConstants.FACE_TOWARD_RED)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(FieldConstants.BLUE_AUDIENCE_SPIKE_C, FieldConstants.FACE_TOWARD_RED)
                .splineToLinearHeading(FieldConstants.BLUE_BACKDROP_CENTER, FieldConstants.FACE_TOWARD_BACKSTAGE)
                .build());

        // Declare out second bot
//        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
//                // We set this bot to be red
//                .setColorScheme(new ColorSchemeRedDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .build();

//        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(30, 30, Math.toRadians(180)))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
//                .addEntity(mySecondBot)
                .start();
    }
}