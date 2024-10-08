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
    public enum AllianceColor {BLUE, RED}
    public enum SideOfField {OBSERVATION, NET}
    public enum TeamPropLocation {LEFT, CENTER, RIGHT}

    // Centralized method to calculate the starting pose based on alliance color and side of the field
    public static Pose2d getStartPose(AllianceColor allianceColor, SideOfField sideOfField) {
        Pose2d baseStartPose;

        // Determine the starting pose based on the side of the field
        if (sideOfField == SideOfField.NET) {
            baseStartPose = NET_START_POSE;
        } else {
            baseStartPose = OBSERVATION_START_POSE;
        }

        // Flip for blue alliance
        if (allianceColor == AllianceColor.BLUE) {
            baseStartPose = rotate(baseStartPose);  // Rotate the pose 180 degrees for blue alliance
        }

        return baseStartPose;
    }


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

    public static Pose2d OBSERVATION_START_POSE = new Pose2d(HALF_TILE,-HALF_FIELD + HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d NET_START_POSE = new Pose2d(-HALF_TILE,  -HALF_FIELD+HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);

    public static Pose2d NET_CHAMBER = new Pose2d(-HALF_TILE, -TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d BASKET_RED_AUDIENCE_2_TJ = new Vector2d(-55.6, -51.6);

    public static Pose2d NET_SPIKE_ONE = new Pose2d(-2*TILE, -TILE-QUARTER_TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d NET_SPIKE_ONE_VEC = PoseToVector(NET_SPIKE_ONE);

    public static Pose2d NET_SPIKE_TWO = new Pose2d(-2*TILE-HALF_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Vector2d NET_SPIKE_TWO_VEC = PoseToVector(NET_SPIKE_TWO);

    public static Pose2d NET_SPIKE_THREE = new Pose2d(-55, -25.5, FACE_TOWARD_AUDIENCE);



    //Need descriptions of these positions, I don't understand what they mean
    public static Vector2d NET_SPIKE_ONE_BEHIND = new Vector2d(-48.1, -48);
    public static Pose2d NET_POS_AUDIENCE_TJ = new Pose2d(-61.8, -52, FACE_TOWARD_BLUE);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ = new Vector2d(-58.5, -48);
    public static Pose2d WALL_ALIGN_POS_AUDIENCE_TJ = new Pose2d(-61.8, -38.6,  FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_NEUTRAL_AUDIENCE_3 = new Pose2d(-2*TILE-HALF_TILE-QUARTER_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Vector2d SPIKE_NEUTRAL_AUDIENCE_3_TJ = new Vector2d(-68, -38.6);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_3_TJ = new Vector2d(-68, -48);
    public static Pose2d NET_ZONE_RED = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Pose2d ASCENT_RED_AUDIENCE = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Vector2d RUNG_RED_BACKSTAGE = new Vector2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH);

    // POSITIONS FOR PLACING SPECIMEN ON CHAMBER DURING AUTO
    public static Pose2d NET_CHAMBER_PRELOAD = new Pose2d(-HALF_TILE, -TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d OBS_CHAMBER_PRELOAD = new Pose2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d OBS_CHAMBER_PRELOAD_VEC = PoseToVector(OBS_CHAMBER_PRELOAD);

    public static Pose2d OBS_CHAMBER_TWO = new Pose2d(HALF_TILE - QUARTER_TILE, -TILE-HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Vector2d OBS_CHAMBER_TWO_VEC = PoseToVector(OBS_CHAMBER_TWO);
    public static Vector2d OBS_CHAMBER_THREE_VEC = new Vector2d(0, -TILE-HALF_ROBOT_LENGTH);
    public static Vector2d OBS_CHAMBER_FOUR_VEC = new Vector2d(HALF_TILE - TILE*0.75, -TILE-HALF_ROBOT_LENGTH);

    //    public static Pose2d OBS_SPIKE_ONE = new Pose2d(2*TILE, -TILE, FACE_TOWARD_BLUE);
//    public static Pose2d OBS_SPIKE_ONE = new Pose2d(2*TILE, -TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
//    public static Vector2d OBS_SPIKE_ONE_VEC = PoseToVector(OBS_SPIKE_ONE);

    public static Pose2d OBS_SPIKE_ONE = flipXAxis(NET_SPIKE_ONE);
    public static Vector2d OBS_SPIKE_ONE_VEC = PoseToVector(OBS_SPIKE_ONE);

    //Finish refactoring these
    public static Pose2d RIGHT_TO_CHAMBER = new Pose2d(TILE+HALF_TILE, -TILE-HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_2 = new Pose2d(2*TILE+HALF_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_3 = new Pose2d(2*TILE+HALF_TILE+QUARTER_TILE, -TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_2_OB = new Pose2d(2*TILE+HALF_TILE, -HALF_TILE-QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_3_OB = new Pose2d(2*TILE+HALF_TILE+QUARTER_TILE, -HALF_TILE-QUARTER_TILE, FACE_TOWARD_BLUE);
    public static Pose2d OBS_WAYPOINT = new Pose2d(46.5,-46,FACE_TOWARD_RED);
    public static Pose2d OBSERVATION_ZONE_RED_PICKUP = new Pose2d(2*TILE, -2*TILE-HALF_TILE, FACE_TOWARD_RED);
    public static Vector2d OBSERVATION_RED_ZONE = new Vector2d(2*TILE, -2*TILE-HALF_TILE);
    public static Pose2d ASCENT_RED_BACKSTAGE = new Pose2d(TILE, -HALF_TILE, FACE_TOWARD_AUDIENCE);
    public static Vector2d NEXT_TO_ASCENT_RED_BACKSTAGE = new Vector2d(TILE+HALF_TILE, -HALF_TILE-QUARTER_TILE);

    public static Pose2d rotate(Pose2d pose) {
        return new Pose2d(-pose.position.x, -pose.position.y, pose.heading.inverse().log());
    }

    public static Pose2d flipXAxis(Pose2d pose) {
        Pose2d output = new Pose2d(-pose.position.x, pose.position.y, pose.heading.log());
        return output;
    }

    public static Pose2d flipYAxis(Pose2d pose) {
        Pose2d output = new Pose2d(pose.position.x, -pose.position.y, pose.heading.log());
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