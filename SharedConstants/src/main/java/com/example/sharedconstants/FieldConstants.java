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
    public enum SampleColor {BLUE, YELLOW, RED, UNKNOWN, NO_SAMPLE}
    public enum SideOfField {OBSERVATION, NET}
    public enum TeamPropLocation {LEFT, CENTER, RIGHT}

    // Centralized method to calculate the starting pose based on alliance color and side of the field
    public static Pose2d getStartPose(SideOfField sideOfField, AllianceColor allianceColor) {
        Pose2d baseStartPose;
        if (sideOfField == SideOfField.NET) {
            baseStartPose = NET_START_POSE;
        } else {
            baseStartPose = OBS_START_POSE;
        }

        //  Flip for blue alliance
        if ( allianceColor== AllianceColor.BLUE) {
            baseStartPose = rotate(baseStartPose);  // Rotate the pose 180 degrees for blue alliance
        }

        return baseStartPose;
    }

    public static double HALF_FIELD = 70.5;
    public static double TILE = 23.5;
    public static double HALF_TILE = TILE/2;
    public static double QUARTER_TILE = TILE/4;
    public static double EIGHTH_TILE = TILE/8;
    public static double THREE_QUARTER_TILE = TILE*.75;
    public static double ROBOT_LENGTH = 16.5;
    public static double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2.0;
    public static double ROBOT_WIDTH = 13.5;
    public static double HALF_ROBOT_WIDTH = ROBOT_WIDTH/2.0;

    public static double SAMPLE_WIDTH = 2.25;
    public static double HALF_SAMPLE_WIDTH= SAMPLE_WIDTH/2.0;
    public static double SAMPLE_LENGTH = 4;
    public static double HALF_SAMPLE_LENGTH = SAMPLE_LENGTH/2.0;

    public static double PICKUP_ROOM = 1.5;

    public static double ANGLE_TOWARD_OBSERVATION = Math.toRadians(0);
    public static double ANGLE_30_DEGREES = Math.toRadians(30);
    public static double ANGLE_45_DEGREES = Math.toRadians(45);
    public static double ANGLE_TOWARD_BLUE = Math.toRadians(90);
    public static double ANGLE_115_DEGREES = Math.toRadians(115);
    public static double ANGLE_135_DEGREES = Math.toRadians(135);
    public static double ANGLE_160_DEGREES = Math.toRadians(160);
    public static double ANGLE_TOWARD_NET = Math.toRadians(180);
    public static double ANGLE_225_DEGREES = Math.toRadians(225);
    public static double ANGLE_TOWARD_RED = Math.toRadians(270);
    public static double ANGLE_315_DEGREES = Math.toRadians(315);
    public static double ANGLE_340_DEGREES = Math.toRadians(340);

    //Chamber Slot Poses
    public static Pose2d CHAMBER_SLOT_ONE = new Pose2d(QUARTER_TILE, -TILE-HALF_ROBOT_LENGTH+1, ANGLE_TOWARD_BLUE);
    public static Pose2d CHAMBER_SLOT_TWO = CHAMBER_SLOT_ONE.plus(new Twist2d(new Vector2d(0,TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_THREE = CHAMBER_SLOT_TWO.plus(new Twist2d(new Vector2d(0, TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_FOUR = CHAMBER_SLOT_THREE.plus(new Twist2d(new Vector2d(0, TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_FIVE = CHAMBER_SLOT_FOUR.plus(new Twist2d(new Vector2d(0,TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_SIX = CHAMBER_SLOT_FIVE.plus(new Twist2d(new Vector2d(0,TILE/6), 0));
    public static Pose2d CHAMBER_SLOT_SEVEN = CHAMBER_SLOT_SIX.plus(new Twist2d(new Vector2d(0,TILE/6), 0));

    public static Pose2d CHAMBER_STAGING_FOR_SCORING = new Pose2d(TILE+HALF_TILE, -TILE-HALF_ROBOT_LENGTH-3, ANGLE_TOWARD_BLUE);
    public static Pose2d CHAMBER_STAGING_FOR_PICKUP =new Pose2d(TILE+HALF_TILE, -TILE-HALF_ROBOT_LENGTH-3, ANGLE_TOWARD_RED);


    //Preload Poses
    public static Pose2d NET_START_POSE = new Pose2d(-HALF_ROBOT_WIDTH,  -HALF_FIELD + HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d NET_START_POSE_WITH_SAMPLE_PRELOAD = new Pose2d(-HALF_ROBOT_LENGTH,-HALF_FIELD + HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);

    public static Pose2d OBS_START_POSE_FORWARD = new Pose2d(HALF_ROBOT_WIDTH,-HALF_FIELD + HALF_ROBOT_LENGTH+12, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_START_POSE = new Pose2d(HALF_ROBOT_WIDTH,-HALF_FIELD + HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_START_POSE_WITH_SAMPLE_PRELOAD = new Pose2d(HALF_ROBOT_LENGTH,-HALF_FIELD + HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);

    public static Pose2d NET_DO_NOTHING = new Pose2d(-HALF_ROBOT_WIDTH,  -HALF_FIELD + HALF_ROBOT_LENGTH+.01, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DO_NOTHING = new Pose2d(HALF_ROBOT_WIDTH,-HALF_FIELD + HALF_ROBOT_LENGTH+.01, ANGLE_TOWARD_BLUE);


    //NET_Preload_and_One_Sample Poses
    public static Pose2d NET_SPIKE_ONE = new Pose2d(-35, -25.5, ANGLE_TOWARD_NET);
    public static Pose2d NET_SPIKE_ONE_WAYPOINT = NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(0,5), 0));
    public static Pose2d NET_SPIKE_ONE_PICKUP = NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(5,0), 0));
    public static Pose2d NET_BASKET = new Pose2d(-54, -54, ANGLE_45_DEGREES);

    //NET_Preload_and_Two_Samples Poses
    public static Pose2d NET_SPIKE_TWO = new Pose2d(-43.5, -25.5, ANGLE_TOWARD_NET);
    public static Pose2d NET_SPIKE_TWO_PICKUP = NET_SPIKE_TWO.plus(new Twist2d(new Vector2d(5,0), 0));

    //NET_Preload_and_Two_Samples Poses
    public static Pose2d NET_SPIKE_THREE = new Pose2d(-55, -25.5, ANGLE_TOWARD_NET);
    public static Pose2d NET_SPIKE_THREE_PICKUP = NET_SPIKE_THREE.plus(new Twist2d(new Vector2d(5,0), 0));

    public static Pose2d HUMAN_PLAYER_SAMPLE_STAGING = new Pose2d(TILE-HALF_ROBOT_LENGTH, -3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d HUMAN_PLAYER_SAMPLE_PICKUP = HUMAN_PLAYER_SAMPLE_STAGING.plus(new Twist2d(new Vector2d(THREE_QUARTER_TILE,0), 0));
    public static Pose2d NET_BASKET_WALL = new Pose2d(-2*TILE, -3*TILE+HALF_ROBOT_LENGTH, ANGLE_TOWARD_OBSERVATION);

    //NET Short Poses
    public static Pose2d NET_BASKET_NEUTRAL_SIDE = new Pose2d(-3*TILE+HALF_ROBOT_WIDTH,-2*TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d NET_SPIKE_ONE_SHORT = new Pose2d(-2*TILE+HALF_ROBOT_WIDTH-2*SAMPLE_WIDTH,-TILE-HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d NET_SPIKE_TWO_SHORT = new Pose2d(-2*TILE+HALF_ROBOT_WIDTH-SAMPLE_WIDTH-HALF_TILE,-TILE-HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d NET_SPIKE_THREE_SHORT = new Pose2d(-3*TILE+HALF_ROBOT_WIDTH,-TILE-HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);


    public static Pose2d NEXT_TO_NET_ASCENT = new Pose2d(-TILE-HALF_TILE, -HALF_TILE, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d NET_ASCENT = new Pose2d(-TILE, -HALF_TILE, ANGLE_TOWARD_OBSERVATION);


    //Observation Poses
    public static Pose2d OBS_WAYPOINT = new Pose2d(2*TILE+HALF_ROBOT_WIDTH,-2*TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_ZONE_PICKUP = new Pose2d(2*TILE+HALF_ROBOT_WIDTH, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_RED);
    public static Pose2d OBS_ZONE_PICKUP_FACE_TOWARD_BLUE = new Pose2d(2*TILE+HALF_ROBOT_WIDTH, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM+1.1, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_ZONE_DUMP = new Pose2d(2*TILE+HALF_ROBOT_WIDTH, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM+HALF_ROBOT_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_ZONE_BEFORE_PICKUP = new Pose2d(2*TILE+HALF_ROBOT_WIDTH, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM+1.1+HALF_TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d RIGHT_OF_CHAMBER =  new Pose2d(TILE+HALF_TILE+1, -TILE-4, ANGLE_TOWARD_RED);
    public static Pose2d RIGHT_OF_CHAMBER_INTAKE_1 =  new Pose2d(TILE+HALF_TILE+1+TILE-EIGHTH_TILE-5, -TILE-4-HALF_ROBOT_LENGTH-QUARTER_TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d RIGHT_OF_CHAMBER_INTAKE_2 =  new Pose2d(TILE+HALF_TILE+1+TILE+EIGHTH_TILE-1, -TILE-4-HALF_ROBOT_LENGTH-SAMPLE_LENGTH, ANGLE_TOWARD_BLUE);
    public static Pose2d RIGHT_OF_CHAMBER_INTAKE_3 =  new Pose2d(TILE+HALF_TILE+1+TILE+EIGHTH_TILE-1-EIGHTH_TILE, -TILE-4-1, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d NEXT_TO_OBS_ASCENT = new Pose2d(TILE+HALF_TILE, -HALF_TILE-QUARTER_TILE, ANGLE_TOWARD_OBSERVATION);

    public static Pose2d OBS_BEHIND_SPIKE_ONE = new Pose2d(2*TILE-QUARTER_TILE+1, -HALF_TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_SPIKE_ONE = new Pose2d(2*TILE, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_ONE = new Pose2d(2*TILE, -2*TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_INTAKE_SPIKE_ONE = new Pose2d(2*TILE, -2*TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_BEHIND_SPIKE_TWO = new Pose2d(2*TILE+QUARTER_TILE, -HALF_TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_SPIKE_TWO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_TWO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH, -2*TILE, ANGLE_TOWARD_RED);

    public static Pose2d OBS_BEHIND_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH-QUARTER_TILE+1, -HALF_TILE, ANGLE_TOWARD_RED);
    public static Pose2d OBS_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH+.5, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH+.5, -TILE*2, ANGLE_TOWARD_RED);
    public static Pose2d OBS_INTAKE_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH+.5, -TILE*2, ANGLE_TOWARD_BLUE);


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