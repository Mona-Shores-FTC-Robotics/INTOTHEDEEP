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


//todo can we add @Config to this and adjust the points in the dashboard to help with debugging quickly?
//todo talk to programming mentors about fast loading - see https://gist.github.com/MatthewOates36/1e38452236dc9f145c3a6b2addfb418f
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

    public static Pose2d getResetCornerPose(AllianceColor allianceColor) {
        Pose2d baseStartPose= OBS_CORNER_RESET_POSE;
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

    public static double PICKUP_ROOM = 2.2;

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
    public static Pose2d CHAMBER_SLOT_ONE = new Pose2d(QUARTER_TILE-2, -TILE-HALF_ROBOT_LENGTH+3, ANGLE_TOWARD_BLUE);
    public static Pose2d CHAMBER_SLOT_TWO = CHAMBER_SLOT_ONE.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH+.3), 0));
    public static Pose2d CHAMBER_SLOT_THREE = CHAMBER_SLOT_TWO.plus(new Twist2d(new Vector2d(0, SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_FOUR = CHAMBER_SLOT_THREE.plus(new Twist2d(new Vector2d(0, SAMPLE_WIDTH+.3), 0));
    public static Pose2d CHAMBER_SLOT_FIVE = CHAMBER_SLOT_FOUR.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_SIX = CHAMBER_SLOT_FIVE.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH+.3), 0));
    public static Pose2d CHAMBER_SLOT_SEVEN = CHAMBER_SLOT_SIX.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_SEVEN_ROTATED = CHAMBER_SLOT_SIX.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), Math.toRadians(5)));
    public static Pose2d CHAMBER_SLOT_EIGHT = CHAMBER_SLOT_SEVEN.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_NINE = CHAMBER_SLOT_EIGHT.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_TEN = CHAMBER_SLOT_NINE.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_ELEVEN = CHAMBER_SLOT_TEN.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));
    public static Pose2d CHAMBER_SLOT_TWELVE = CHAMBER_SLOT_ELEVEN.plus(new Twist2d(new Vector2d(0,SAMPLE_WIDTH), 0));


    public static double CHAMBER_SPACER = 1.85;

    public static Pose2d CHAMBER_SLOT_ONE_REDO = new Pose2d(QUARTER_TILE-.8, -TILE-HALF_ROBOT_LENGTH+1.8, ANGLE_TOWARD_BLUE);
    public static Pose2d CHAMBER_SLOT_TWO_REDO = CHAMBER_SLOT_ONE_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_THREE_REDO = CHAMBER_SLOT_TWO_REDO.plus(new Twist2d(new Vector2d(0, CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_FOUR_REDO = CHAMBER_SLOT_THREE_REDO.plus(new Twist2d(new Vector2d(0, CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_FIVE_REDO = CHAMBER_SLOT_FOUR_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_SIX_REDO = CHAMBER_SLOT_FIVE_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_SEVEN_REDO = CHAMBER_SLOT_SIX_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_EIGHT_REDO = CHAMBER_SLOT_SEVEN_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_NINE_REDO = CHAMBER_SLOT_EIGHT_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), Math.toRadians(10)));
    public static Pose2d CHAMBER_SLOT_TEN_REDO = CHAMBER_SLOT_NINE_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_ELEVEN_REDO = CHAMBER_SLOT_TEN_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));
    public static Pose2d CHAMBER_SLOT_TWELVE_REDO = CHAMBER_SLOT_ELEVEN_REDO.plus(new Twist2d(new Vector2d(0,CHAMBER_SPACER), 0));

    public static Pose2d RIGHT_OF_CHAMBER_REDO =  new Pose2d(TILE+HALF_TILE-4, -TILE-EIGHTH_TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_BEHIND_SPIKE_ONE_REDO = new Pose2d(2*TILE, -HALF_TILE-2, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_ONE_REDO = new Pose2d(2*TILE, -2*TILE-3, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_SPIKE_ONE_REDO = new Pose2d(2*TILE, -TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_BEHIND_SPIKE_TWO_REDO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH, -HALF_TILE-3, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_TWO_REDO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH, -2*TILE-3, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_SPIKE_TWO_REDO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH, -TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_BEHIND_SPIKE_THREE_REDO = new Pose2d(3*TILE-HALF_ROBOT_WIDTH, -HALF_TILE-2, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_THREE_REDO = new Pose2d(3*TILE-HALF_ROBOT_WIDTH, -2*TILE, ANGLE_TOWARD_BLUE);

    public static Pose2d NET_START_POSE = new Pose2d(HALF_ROBOT_LENGTH-TILE,  -HALF_FIELD + HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d NET_DO_NOTHING = new Pose2d(NET_START_POSE.position.plus(new Vector2d(0, 0.01)), ANGLE_TOWARD_BLUE);

    public static Pose2d NET_SPIKE_ONE = new Pose2d(-2*TILE+HALF_ROBOT_WIDTH-2*SAMPLE_WIDTH+1,-TILE-HALF_ROBOT_LENGTH-QUARTER_TILE-EIGHTH_TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d NET_SPIKE_ONE_APPROACH = NET_SPIKE_ONE.plus(new Twist2d(new Vector2d(-6,0), 0));

    public static Pose2d NET_SPIKE_TWO = new Pose2d(-2*TILE+HALF_ROBOT_WIDTH-SAMPLE_WIDTH-HALF_TILE,-TILE-HALF_ROBOT_LENGTH-QUARTER_TILE-EIGHTH_TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d NET_SPIKE_TWO_APPROACH = NET_SPIKE_TWO.plus(new Twist2d(new Vector2d(-6,0), 0));

    public static Pose2d NET_SPIKE_THREE = new Pose2d(-3*TILE+HALF_ROBOT_WIDTH,-TILE-HALF_ROBOT_LENGTH-QUARTER_TILE-EIGHTH_TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d NET_SPIKE_THREE_APPROACH = NET_SPIKE_THREE.plus(new Twist2d(new Vector2d(-6,0), 0));

    public static Pose2d NET_BASKET = new Pose2d(-59, -61, ANGLE_45_DEGREES);
    public static Pose2d NET_BASKET_PRE_SCORE = new Pose2d(NET_BASKET.position.x+15, NET_BASKET.position.y+15, ANGLE_45_DEGREES); //-48 -52

    public static Pose2d NET_BASKET_DRIVE_TO_NET_SCORE = new Pose2d(-61.5, -57, Math.toRadians(47));
    public static Pose2d NET_BASKET_DRIVE_TO_NET_APPROACH = new Pose2d(NET_BASKET_DRIVE_TO_NET_SCORE.position.x+10, NET_BASKET_DRIVE_TO_NET_SCORE.position.y+10, Math.toRadians(55)); //-48 -52

    public static Pose2d NET_BASKET_AUTO = new Pose2d(-59, -61, Math.toRadians(55)); //-48 -52
    public static Pose2d NET_BASKET_ALIGNMENT_AUTO = new Pose2d(NET_BASKET_AUTO.position.x+5, NET_BASKET_AUTO.position.y+5, Math.toRadians(55)); //-48 -5


    public static Pose2d NEXT_TO_NET_ASCENT = new Pose2d(-TILE-HALF_TILE, -HALF_TILE, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d NET_ASCENT = new Pose2d(-TILE, -HALF_TILE, ANGLE_TOWARD_OBSERVATION);


    //SHOULD BE ABLE TO DELETE THESE
    public static Pose2d NET_BASKET_NEUTRAL_SIDE = new Pose2d(-3*TILE+HALF_ROBOT_WIDTH,-2*TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d HUMAN_PLAYER_SAMPLE_STAGING = new Pose2d(TILE-HALF_ROBOT_LENGTH, -3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d HUMAN_PLAYER_SAMPLE_PICKUP = HUMAN_PLAYER_SAMPLE_STAGING.plus(new Twist2d(new Vector2d(THREE_QUARTER_TILE,0), 0));
    public static Pose2d PARTNER_PRELOAD_LEFT_BEHIND = new Pose2d(-HALF_TILE, -3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);

    //Observation Poses
    public static Pose2d OBS_START_POSE = new Pose2d(TILE-HALF_ROBOT_LENGTH,-HALF_FIELD + HALF_ROBOT_WIDTH, ANGLE_TOWARD_OBSERVATION);
    public static Pose2d OBS_DO_NOTHING = new Pose2d(OBS_START_POSE.position.plus(new Vector2d(0, 0.01)), ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_ZONE_PICKUP = new Pose2d(2*TILE+HALF_ROBOT_WIDTH, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_TRIANGLE_APPROACH = new Pose2d(TILE+HALF_TILE,-3*TILE+HALF_ROBOT_LENGTH+3*PICKUP_ROOM, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_TRIANGLE_PICKUP = new Pose2d(TILE+HALF_TILE, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_CORNER_APPROACH_ALLIANCE_WALL = new Pose2d(3*TILE-HALF_ROBOT_WIDTH,-3*TILE+HALF_ROBOT_LENGTH+5*PICKUP_ROOM, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_CORNER_PICKUP_ALLIANCE_WALL = new Pose2d(3*TILE-HALF_ROBOT_WIDTH,-3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_CORNER_STAGING_AUDIENCE_WALL = new Pose2d(2*TILE+HALF_TILE -HALF_ROBOT_LENGTH-QUARTER_TILE,-3*TILE+ROBOT_WIDTH, ANGLE_TOWARD_NET);
    public static Pose2d OBS_CORNER_APPROACH_AUDIENCE_WALL = new Pose2d(3*TILE-HALF_ROBOT_LENGTH-QUARTER_TILE,-3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_NET);
    public static Pose2d OBS_CORNER_PICKUP_AUDIENCE_WALL = new Pose2d(3*TILE-HALF_ROBOT_LENGTH-PICKUP_ROOM,-3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_NET);

    public static Pose2d OBS_CORNER_APPROACH_DRIVE_TO_OBS_AUDIENCE_WALL = new Pose2d(3*TILE-HALF_ROBOT_LENGTH-HALF_TILE,-3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_NET);


    public static Pose2d OBS_TRIANGLE_APPROACH_REDO = new Pose2d(2*TILE-HALF_ROBOT_WIDTH+2,-3*TILE+HALF_ROBOT_LENGTH+5*PICKUP_ROOM, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_TRIANGLE_PICKUP_REDO = new Pose2d(2*TILE-HALF_ROBOT_WIDTH+2, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_CORNER_RESET_POSE = new Pose2d(3*TILE-HALF_ROBOT_LENGTH,-3*TILE+HALF_ROBOT_WIDTH, ANGLE_TOWARD_NET);


    public static Pose2d OBS_TRIANGLE_APPROACH_TILE_SEAM = new Pose2d(2*TILE-HALF_ROBOT_WIDTH+2,-3*TILE+HALF_ROBOT_LENGTH+5*PICKUP_ROOM, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_TRIANGLE_PICKUP_TILE_SEAM = new Pose2d(2*TILE-HALF_ROBOT_WIDTH+2, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_TRIANGLE_TIP_APPROACH = new Pose2d(2*TILE-HALF_TILE-HALF_ROBOT_WIDTH+2.5,-3*TILE+HALF_ROBOT_LENGTH+5*PICKUP_ROOM, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_TRIANGLE_TIP_PICKUP = new Pose2d(2*TILE-HALF_TILE-HALF_ROBOT_WIDTH+2.5, -3*TILE+HALF_ROBOT_LENGTH+PICKUP_ROOM, ANGLE_TOWARD_BLUE);

    public static Pose2d RIGHT_OF_CHAMBER =  new Pose2d(TILE+HALF_TILE+1, -TILE-4, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_BEHIND_SPIKE_ONE = new Pose2d(2*TILE-5, -HALF_TILE-2, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_SPIKE_ONE = new Pose2d(2*TILE+4, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_ONE = new Pose2d(2*TILE+4, -2*TILE-4, ANGLE_TOWARD_BLUE);


    public static Pose2d OBS_BEHIND_SPIKE_TWO = new Pose2d(2*TILE+QUARTER_TILE+2, -HALF_TILE-3, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_SPIKE_TWO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH+1.2, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_TWO = new Pose2d(2*TILE+HALF_TILE-HALF_SAMPLE_WIDTH+1.2, -TILE-HALF_TILE-QUARTER_TILE-4, ANGLE_TOWARD_BLUE);

    public static Pose2d OBS_BEHIND_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH-QUARTER_TILE+1, -HALF_TILE-3, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH+.5, -TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d OBS_DELIVER_SPIKE_THREE = new Pose2d(3*TILE-HALF_ROBOT_WIDTH+.5, -TILE-HALF_TILE-QUARTER_TILE-2, ANGLE_TOWARD_BLUE);

    public static Pose2d SPIKE_1_GROUND_PICKUP =  new Pose2d(2*TILE+HALF_TILE-EIGHTH_TILE-2, -TILE-4-HALF_ROBOT_LENGTH-QUARTER_TILE, ANGLE_TOWARD_BLUE);
    public static Pose2d SPIKE_2_GROUND_PICKUP =  new Pose2d(TILE+HALF_TILE+1+TILE+EIGHTH_TILE-1, -TILE-4-HALF_ROBOT_LENGTH-QUARTER_TILE, ANGLE_TOWARD_BLUE);


    public static Pose2d AUTO_TEST_POSE = new Pose2d(-2*TILE,  -2*TILE, ANGLE_TOWARD_BLUE);




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