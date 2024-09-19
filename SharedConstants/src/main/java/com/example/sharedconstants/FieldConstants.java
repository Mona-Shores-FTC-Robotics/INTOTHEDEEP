// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

// TODO Field Constants - improve our constants class
//  - should we have Pose2d and Vector for every point of interest?
//  - should we change how we name our "facings"?

public class FieldConstants {
    public static enum AllianceColor {BLUE, RED}
    public static enum SideOfField {BACKSTAGE, AUDIENCE}
    public static enum TeamPropLocation {LEFT, CENTER, RIGHT}

    public static double END_GAME_TIME=90;

    public static double HALF_FIELD = 72.0;
    public static double TILE = 23.5;
    public static double HALF_TILE = TILE/2;
    public static double QUARTER_TILE = TILE/4;
    public static double EIGHTH_TILE = TILE/8;
    public static double THREE_QUARTER_TILE = TILE*.75;
    public static double ROBOT_LENGTH = 18.0;
    public static double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2;

    public static double FACE_TOWARD_BACKSTAGE = Math.toRadians(0);
    public static double FACE_30_DEGREES = Math.toRadians(30);
    public static double FACE_45_DEGREES = Math.toRadians(45);
    public static double FACE_TOWARD_BLUE = Math.toRadians(90);
    public static double FACE_115_DEGREES = Math.toRadians(115);
    public static double FACE_135_DEGREES = Math.toRadians(135);
    public static double FACE_160_DEGREES = Math.toRadians(160);
    public static double FACE_TOWARD_AUDIENCE = Math.toRadians(180);
    public static double FACE_225_DEGREES = Math.toRadians(225);
    public static double FACE_TOWARD_RED = Math.toRadians(270);
    public static double FACE_315_DEGREES = Math.toRadians(315);
    public static double FACE_340_DEGREES = Math.toRadians(340);

    public static double TANGENT_TOWARD_BACKSTAGE = Math.toRadians(0);
    public static double TANGENT_45_DEGREES = Math.toRadians(45);
    public static double TANGENT_TOWARD_BLUE = Math.toRadians(90);
    public static double TANGENT_135_DEGREES = Math.toRadians(135);
    public static double TANGENT_TOWARD_AUDIENCE = Math.toRadians(180);
    public static double TANGENT_225_DEGREES = Math.toRadians(225);
    public static double TANGENT_TOWARD_RED = Math.toRadians(270);
    public static double TANGENT_315_DEGREES = Math.toRadians(315);

    public static Pose2d BLUE_AUDIENCE_START_POSE = new Pose2d(-HALF_TILE, HALF_FIELD-HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
    public static Pose2d RED_AUDIENCE_START_POSE = new Pose2d(-HALF_TILE,  -HALF_FIELD+HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE,-HALF_FIELD + HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE, HALF_FIELD-HALF_ROBOT_LENGTH, FACE_TOWARD_RED);

    //Red Points of Interest
    public static Pose2d CHAMBER_RED_AUDIENCE = new Pose2d(-HALF_TILE, -TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d BASKET_RED_AUDIENCE_2_TJ = new Vector2d(-55.6, -51.6);
    public static Pose2d SPIKE_NEUTRAL_AUDIENCE_3_POS_TJ = new Pose2d(-55, -25.5, FACE_TOWARD_AUDIENCE);

    public static Pose2d SPIKE_NEUTRAL_AUDIENCE_1 = new Pose2d(-2*TILE, -TILE-QUARTER_TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d SPIKE_NEUTRAL_AUDIENCE_1_TJ = new Vector2d(-48.1, -38.6);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_1_TJ = new Vector2d(-48.1, -48);

    public static Pose2d SPIKE_NEUTRAL_AUDIENCE_2 = new Pose2d(-2*TILE-HALF_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Pose2d NET_POS_AUDIENCE_TJ = new Pose2d(-61.8, -52, FACE_TOWARD_BLUE);
    public static Vector2d SPIKE_NEUTRAL_AUDIENCE_2_TJ = new Vector2d(-58.5, -38.6);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ = new Vector2d(-58.5, -48);

    public static Pose2d SPIKE_NEUTRAL_AUDIENCE_3 = new Pose2d(-2*TILE-HALF_TILE-QUARTER_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Vector2d SPIKE_NEUTRAL_AUDIENCE_3_TJ = new Vector2d(-68, -38.6);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_3_TJ = new Vector2d(-68, -48);

    public static Pose2d NET_ZONE_RED = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Pose2d ASCENT_RED_AUDIENCE = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);

    public static Vector2d RUNG_RED_BACKSTAGE = new Vector2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH);
    public static Pose2d WALL_ALIGN_POS_AUDIENCE_TJ = new Pose2d(-61.8, -38.6,  FACE_TOWARD_BLUE);
   // public static Vector2d NET_POS_AUDIENCE_TJ = new Vector2d(-61.8, -52);
    public static Pose2d CHAMBER_RED_BACKSTAGE = new Pose2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d CHAMBER_RED_BACKSTAGE_VEC = new Vector2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH);
    public static Pose2d SPIKE_RED_1 = new Pose2d(2*TILE, -TILE, FACE_TOWARD_BLUE);
    public static Vector2d SPIKE_RED_1_Vec = new Vector2d(48.1, -38.6);

    public static Pose2d SPIKE_RED_1_OB = new Pose2d(2*TILE, -TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d RIGHT_TO_CHAMBER = new Pose2d(TILE+HALF_TILE, -TILE-HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_2 = new Pose2d(2*TILE+HALF_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_3 = new Pose2d(2*TILE+HALF_TILE+QUARTER_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_2_OB = new Pose2d(2*TILE+HALF_TILE, -HALF_TILE-QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_3_OB = new Pose2d(2*TILE+HALF_TILE+QUARTER_TILE, -HALF_TILE-QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Vector2d OBSERVATION_RED_ZONE = new Vector2d(2*TILE, -2*TILE-HALF_TILE);
    public static Pose2d ASCENT_RED_BACKSTAGE = new Pose2d(TILE, -HALF_TILE, FACE_TOWARD_AUDIENCE);
    public static Vector2d NEXT_TO_ASCENT_RED_BACKSTAGE = new Vector2d(TILE+HALF_TILE, -HALF_TILE-QUARTER_TILE);



    //Blue points of interest, should not be needed
//    public static Pose2d NET_ZONE_BLUE = new Pose2d(2*TILE, 2*TILE, FACE_45_DEGREES);
//    public static Pose2d OBSERVATION_BLUE_ZONE = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
//
//    public static Pose2d ASCENT_BLUE_BACKSTAGE = new Pose2d(TILE, HALF_TILE, FACE_TOWARD_AUDIENCE);
//    public static Pose2d ASCENT_BLUE_AUDIENCE = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
//
//    public static Pose2d RUNG_BLUE_AUDIENCE = new Pose2d(-HALF_TILE, TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
//    public static Pose2d RUNG_BLUE_BACKSTAGE = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
//
//    public static Pose2d SPIKE_BLUE_1 = new Pose2d(-2*TILE, TILE, FACE_TOWARD_RED);
//    public static Pose2d SPIKE_BLUE_2 = new Pose2d(-2*TILE-HALF_TILE, TILE, FACE_TOWARD_RED);
//    public static Pose2d SPIKE_BLUE_3 = new Pose2d(-2*TILE-HALF_TILE-QUARTER_TILE, TILE, FACE_TOWARD_RED);
//    public static Pose2d SPIKE_NEUTRAL_BACKSTAGE_1 = new Pose2d(2*TILE, TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
//    public static Pose2d SPIKE_NEUTRAL_BACKSTAGE_2 = new Pose2d(2*TILE+HALF_TILE, TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
//    public static Pose2d SPIKE_NEUTRAL_BACKSTAGE_3 = new Pose2d(2*TILE+HALF_TILE+QUARTER_TILE, TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_RED);




    public static Pose2d flipYAxis(Pose2d pose) {
        Pose2d output = new Pose2d(pose.position.x, -pose.position.y, Math.toRadians(pose.heading.imag+pose.heading.real));
        return output;
    }

    public static Vector2d flipYAxis(Vector2d vector) {
        Vector2d output = new Vector2d(vector.x, -vector.y);
        return output;
    }

    public static Vector2d PoseToVector(Pose2d pose){
        return new Vector2d (pose.position.x, pose.position.y);



    }
}