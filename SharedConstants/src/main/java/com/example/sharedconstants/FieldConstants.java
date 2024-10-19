// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package com.example.sharedconstants;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldConstants {
    public enum AllianceColor {BLUE, RED}
    public enum SideOfField {OBSERVATION, NET}
    public enum TeamPropLocation {LEFT, CENTER, RIGHT}

    // Centralized method to calculate the starting pose based on alliance color and side of the field
    public static Pose2d getStartPose(SideOfField sideOfField) {
        Pose2d baseStartPose;
        if (sideOfField == SideOfField.NET) {
            baseStartPose = NET_START_POSE;
        } else {
            baseStartPose = OBS_START_POSE;
        }
        return baseStartPose;
    }

    public static double HALF_FIELD = 70.5;
    public static double TILE = 23.5;
    public static double HALF_TILE = TILE/2;
    public static double QUARTER_TILE = TILE/4;
    public static double EIGHTH_TILE = TILE/8;
    public static double THREE_QUARTER_TILE = TILE*.75;
    public static double ROBOT_LENGTH = 18.0;
    public static double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2.0;
    public static double ROBOT_WIDTH = 18.0;
    public static double HALF_ROBOT_WIDTH = ROBOT_LENGTH/2.0;

    public static double PICKUP_ROOM = 1.5;

    public static double FACE_TOWARD_BACKSTAGE = Math.toRadians(0);
    public static double FACE_30_DEGREES = Math.toRadians(30);
    public static double FACE_45_DEGREES = Math.toRadians(45);
    public static double ANGLE_TOWARD_BLUE = Math.toRadians(90);
    public static double FACE_115_DEGREES = Math.toRadians(115);
    public static double FACE_135_DEGREES = Math.toRadians(135);
    public static double FACE_160_DEGREES = Math.toRadians(160);
    public static double FACE_TOWARD_AUDIENCE = Math.toRadians(180);
    public static double FACE_225_DEGREES = Math.toRadians(225);
    public static double ANGLE_TOWARD_RED = Math.toRadians(270);
    public static double FACE_315_DEGREES = Math.toRadians(315);
    public static double FACE_340_DEGREES = Math.toRadians(340);

    //Chamber Slot Poses
    public static Pose2d CHAMBER_SLOT_ONE = new Pose2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d CHAMBER_SLOT_TWO = CHAMBER_SLOT_ONE.plus(new Twist2d(new Vector2d(0,TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_THREE = CHAMBER_SLOT_TWO.plus(new Twist2d(new Vector2d(0, TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_FOUR = CHAMBER_SLOT_THREE.plus(new Twist2d(new Vector2d(0, TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_FIVE = CHAMBER_SLOT_FOUR.plus(new Twist2d(new Vector2d(0,TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_SIX = CHAMBER_SLOT_FIVE.plus(new Twist2d(new Vector2d(0,TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_SEVEN = CHAMBER_SLOT_SIX.plus(new Twist2d(new Vector2d(0,TILE/6), 0));

    //Preload Poses
    public static Pose2d NET_START_POSE = new Pose2d(-HALF_ROBOT_WIDTH,  -HALF_FIELD + HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_START_POSE = new Pose2d(HALF_ROBOT_WIDTH,-HALF_FIELD + HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);

    //NET_Preload_and_One_Sample Poses
    public static Pose2d NET_SPIKE_ONE = new Pose2d(-35, -25.5, FACE_TOWARD_AUDIENCE);
    public static Pose2d NET_SPIKE_ONE_WAYPOINT = NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(0,5), 0));
    public static Pose2d NET_SPIKE_ONE_PICKUP = NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(5,0), 0));
    public static Pose2d NET_BASKET = new Pose2d(-54, -54, FACE_45_DEGREES);

    //NET_Preload_and_Two_Samples Poses
    public static Pose2d NET_SPIKE_TWO = new Pose2d(-43.5, -25.5, FACE_TOWARD_AUDIENCE);
    public static Pose2d NET_SPIKE_TWO_PICKUP = NET_SPIKE_TWO.plus(new Twist2d(new Vector2d(5,0), 0));

    //NET_Preload_and_Two_Samples Poses
    public static Pose2d NET_SPIKE_THREE = new Pose2d(-55, -25.5, FACE_TOWARD_AUDIENCE);
    public static Pose2d NET_SPIKE_THREE_PICKUP = NET_SPIKE_THREE.plus(new Twist2d(new Vector2d(5,0), 0));

    //Observation Poses
    public static Pose2d OBS_CHAMBER_THREE = new Pose2d(0, -TILE-HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_WAYPOINT = new Pose2d(2*TILE,-2*TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_ZONE_PICKUP = new Pose2d(2*TILE, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_RED);
    public static Pose2d OBS_BEHIND_SPIKE_ONE = new Pose2d(2*TILE-5, -HALF_TILE, ANGLE_TOWARD_RED);
//    public static Vector2d OBSERVATION_RED_ZONE = new Vector2d(2*TILE, -2*TILE-HALF_TILE);


    public static Pose2d RIGHT_OF_CHAMBER = new Pose2d(TILE+HALF_TILE, -TILE-HALF_ROBOT_WIDTH, ANGLE_TOWARD_RED);
    public static Pose2d NEXT_TO_OBS_ASCENT = new Pose2d(TILE+HALF_TILE, -HALF_TILE-QUARTER_TILE, FACE_TOWARD_BACKSTAGE);


    public static Pose2d OBS_BEHIND_SPIKE_TWO = new Pose2d(2*TILE+HALF_TILE-2, -HALF_TILE-QUARTER_TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_DELIVER_SPIKE_TWO = new Pose2d(2*TILE+HALF_TILE-2, -TILE*2, ANGLE_TOWARD_RED);

    public static Pose2d OBS_BEHIND_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH, -HALF_TILE-QUARTER_TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_DELIVER_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH, -TILE*2, ANGLE_TOWARD_RED);


    //Need descriptions of these positions, I don't understand what they mean
    public static Pose2d NET_SPIKE_TEST_TURN = new Pose2d(-48.1, -36.7, FACE_225_DEGREES);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_2_TJ = new Vector2d(-58.5, -48);
    public static Pose2d WALL_ALIGN_POS_AUDIENCE_TJ = new Pose2d(-61.8, -38.6, ANGLE_TOWARD_BLUE);
    public static Pose2d SPIKE_NEUTRAL_AUDIENCE_3 = new Pose2d(-2*TILE-HALF_TILE-QUARTER_TILE, -TILE, ANGLE_TOWARD_BLUE);
    public static Vector2d SPIKE_NEUTRAL_AUDIENCE_3_TJ = new Vector2d(-68, -38.6);
    public static Vector2d SPIKE_BEHIND_NEUTRAL_AUDIENCE_3_TJ = new Vector2d(-68, -48);
    public static Pose2d NET_ZONE_RED = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Pose2d ASCENT_RED_AUDIENCE = new Pose2d(-TILE-QUARTER_TILE, -TILE-HALF_TILE, FACE_45_DEGREES);
    public static Vector2d RUNG_RED_BACKSTAGE = new Vector2d(HALF_TILE, -TILE-HALF_ROBOT_LENGTH);
    public static Pose2d NET_ZONE_SCORE = new Pose2d(-55.5, -52.2, FACE_45_DEGREES);

    // POSITIONS FOR PLACING SPECIMEN ON CHAMBER DURING AUTO


    public static Vector2d OBS_CHAMBER_TWO_VEC = PoseToVector(CHAMBER_SLOT_TWO);

    public static Vector2d OBS_CHAMBER_THREE_VEC = new Vector2d(0, -TILE-HALF_ROBOT_LENGTH);
    public static Vector2d OBS_CHAMBER_FOUR_VEC = new Vector2d(HALF_TILE - TILE*0.75, -TILE-HALF_ROBOT_LENGTH);

    //    public static Pose2d OBS_SPIKE_ONE = new Pose2d(2*TILE, -TILE, FACE_TOWARD_BLUE);
//    public static Pose2d OBS_SPIKE_ONE = new Pose2d(2*TILE, -TILE+HALF_ROBOT_LENGTH, FACE_TOWARD_BLUE);
//    public static Vector2d OBS_SPIKE_ONE_VEC = PoseToVector(OBS_SPIKE_ONE);

    //Finish refactoring these
    public static Pose2d SPIKE_RED_2 = new Pose2d(2*TILE+HALF_TILE, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d SPIKE_RED_3 = new Pose2d(2*TILE+HALF_TILE+QUARTER_TILE, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d ASCENT_RED_BACKSTAGE = new Pose2d(TILE, -HALF_TILE, FACE_TOWARD_AUDIENCE);

    public static Pose2d rotate(Pose2d pose) {
        return new Pose2d(-pose.position.x, -pose.position.y, pose.heading.plus(Math.toRadians(180)).log());
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