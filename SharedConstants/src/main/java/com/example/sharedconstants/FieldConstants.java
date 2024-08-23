// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldConstants {
    //bannana
    public static double END_GAME_TIME=90;

    public static double ROBOT_LENGTH = 18.0;
    public static double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2;

    public static double HALF_FIELD = 72.0;
    public static double TILE = 23.5;
    public static double HALF_TILE = TILE/2;
    public static double QUARTER_TILE = TILE/4;
    public static double EIGHTH_TILE = TILE/8;
    public static double THREE_QUARTER_TILE = TILE*.75;

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

    public static Pose2d RED_BACKDROP_STAGING = new Pose2d(2*TILE-10, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_STAGING = new Pose2d(2*TILE-10, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_NEUTRAL_STAGING = new Pose2d(-2*TILE+10, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_NEUTRAL_STAGING = new Pose2d(-2*TILE+10, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_BACKDROP_RIGHT = new Pose2d(2*TILE, TILE+HALF_TILE-QUARTER_TILE+1.5, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_CENTER = new Pose2d(2*TILE, TILE+HALF_TILE+2, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKDROP_LEFT = new Pose2d(2*TILE, TILE+HALF_TILE+QUARTER_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_BACKDROP_RIGHT = new Pose2d(2*TILE, -TILE-TILE+QUARTER_TILE-4, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP_CENTER = new Pose2d(2*TILE, -TILE-HALF_TILE-2, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKDROP_LEFT = new Pose2d(2*TILE, -TILE-QUARTER_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d BLUE_AUDIENCE_START_POSE = new Pose2d(-HALF_TILE-TILE, HALF_FIELD-HALF_ROBOT_LENGTH, FACE_TOWARD_RED);

    public static Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE, HALF_FIELD-HALF_ROBOT_LENGTH, FACE_TOWARD_RED);
    public static Pose2d BLUE_BACKSTAGE_START_LANE_A = new Pose2d(TILE, HALF_FIELD-THREE_QUARTER_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(HALF_TILE,-HALF_FIELD + HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
    public static Pose2d RED_BACKSTAGE_START_LANE_F = new Pose2d(TILE, -HALF_FIELD+THREE_QUARTER_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_AUDIENCE_START_POSE = new Pose2d(-HALF_TILE-TILE,  -HALF_FIELD+HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);

    public static Pose2d RED_BACKSTAGE_SPIKE_R = new Pose2d(TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Pose2d RED_BACKSTAGE_SPIKE_C = new Pose2d(HALF_TILE, -TILE-HALF_TILE+2, FACE_TOWARD_BLUE);
    public static Pose2d RED_BACKSTAGE_SPIKE_L = new Pose2d(TILE-THREE_QUARTER_TILE, -TILE-HALF_TILE, FACE_135_DEGREES);

    public static Pose2d BLUE_BACKSTAGE_SPIKE_L = new Pose2d(TILE-QUARTER_TILE, TILE+HALF_TILE, FACE_315_DEGREES);
    public static Pose2d BLUE_BACKSTAGE_SPIKE_C = new Pose2d(HALF_TILE, TILE+HALF_TILE+2, FACE_TOWARD_RED);
    public static Pose2d BLUE_BACKSTAGE_SPIKE_R = new Pose2d(TILE-THREE_QUARTER_TILE, TILE+HALF_TILE, FACE_225_DEGREES);
//    public static Pose2d BLUE_BACKSTAGE_SPIKE_R = new Pose2d(TILE-THREE_QUARTER_TILE, TILE+HALF_TILE-1, Math.toRadians(245));

    public static Pose2d BLUE_BACKSTAGE_ALIGNMENT = new Pose2d(TILE, TILE+HALF_TILE+5, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_AUDIENCE_ALIGNMENT = new Pose2d(-TILE-HALF_TILE, TILE+HALF_TILE+3, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_AUDIENCE_SPIKE_R = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Pose2d RED_AUDIENCE_SPIKE_C = new Pose2d(-TILE-HALF_TILE, -TILE-HALF_TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_AUDIENCE_SPIKE_L = new Pose2d(-TILE-THREE_QUARTER_TILE, -TILE-HALF_TILE, FACE_135_DEGREES);

    public static Pose2d BLUE_AUDIENCE_SPIKE_L = new Pose2d(-TILE-QUARTER_TILE, TILE+HALF_TILE, FACE_315_DEGREES);
    public static Pose2d BLUE_AUDIENCE_SPIKE_C = new Pose2d(-TILE-HALF_TILE, TILE+HALF_TILE, FACE_TOWARD_RED);
    public static Pose2d BLUE_AUDIENCE_SPIKE_R = new Pose2d(-TILE-THREE_QUARTER_TILE, TILE+HALF_TILE, FACE_225_DEGREES);

    public static Pose2d RED_AUDIENCE_PARK_LANE_F = new Pose2d(-TILE-HALF_TILE, -2*TILE, FACE_TOWARD_BLUE);
    public static Pose2d RED_AUDIENCE_PARK_LANE_D = new Pose2d(-2*TILE, -HALF_TILE, TANGENT_135_DEGREES);

    public static Pose2d RED_BACKSTAGE_PARK_LANE_F = new Pose2d(2*TILE, -TILE*2-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_BACKSTAGE_PARK_LANE_D = new Pose2d(2*TILE, -HALF_TILE, TANGENT_135_DEGREES);
    public static Pose2d BLUE_BACKSTAGE_PARK_LANE_A = new Pose2d(2*TILE, TILE*2+HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_BACKSTAGE_PARK_LANE_C = new Pose2d(2*TILE, HALF_TILE, FACE_225_DEGREES);

    public static Pose2d RED_STAGEDOOR_ENTRANCE = new Pose2d(-QUARTER_TILE-TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    //public static Pose2d BLUE_STAGEDOOR_ENTRANCE = new Pose2d(-QUARTER_TILE-TILE, HALF_TILE, FACE_TOWARD_AUDIENCE);
    public static Pose2d BLUE_STAGEDOOR_ENTRANCE = new Pose2d(-QUARTER_TILE-TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_STAGEDOOR_EXIT = new Pose2d(HALF_TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_STAGEDOOR_EXIT = new Pose2d(HALF_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_NEUTRAL_PIXEL_STAGEDOOR = new Pose2d(-TILE*2-QUARTER_TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_NEUTRAL_PIXEL_CENTERSPIKE = new Pose2d(-TILE*2-QUARTER_TILE, -TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_NEUTRAL_PIXEL_WING = new Pose2d(-TILE*2-6, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_NEUTRAL_PIXEL_PICKUP = new Pose2d(-TILE*2-HALF_TILE, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_TRUSS = new Pose2d(-HALF_TILE,-TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_TRUSS = new Pose2d(-HALF_TILE,+TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d RED_SPIKE_L_LINE =  new Pose2d(-TILE*2, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_SPIKE_R_LINE =  new Pose2d(-TILE*2, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);



    public static Pose2d BLUE_NEUTRAL_PIXEL_STAGEDOOR = flipYAxis(RED_NEUTRAL_PIXEL_STAGEDOOR);
    public static Pose2d BLUE_NEUTRAL_PIXEL_CENTERSPIKE = flipYAxis(RED_NEUTRAL_PIXEL_CENTERSPIKE);
    public static Pose2d BLUE_NEUTRAL_PIXEL_WING =  new Pose2d(-TILE*2-6, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_NEUTRAL_PIXEL_PICKUP =  new Pose2d(-TILE*2-HALF_TILE, TILE+HALF_TILE, FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_SAFE_STRAFE = new Pose2d(-TILE*2-QUARTER_TILE, -TILE-HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_SAFE_STRAFE = new Pose2d(flipYAxis(new Vector2d(RED_SAFE_STRAFE.position.x, RED_SAFE_STRAFE.position.y)), FACE_TOWARD_BACKSTAGE);

    public static Pose2d RED_THROUGH_DOOR = new Pose2d(TILE+EIGHTH_TILE, -HALF_TILE, FACE_TOWARD_BACKSTAGE);
    public static Pose2d BLUE_THROUGH_DOOR =  new Pose2d(TILE+EIGHTH_TILE, HALF_TILE, FACE_TOWARD_BACKSTAGE);

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