package com.example.sharedconstants.Routes;

import static com.example.sharedconstants.FieldConstants.*;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.example.sharedconstants.RobotDriveAdapter;

public class RoutesSpikeBackdropPark {

    private final RobotDriveAdapter roadRunnerDrive;

    public RoutesSpikeBackdropPark(RobotDriveAdapter roadRunnerDrive) {
        this.roadRunnerDrive = roadRunnerDrive;
    }

    //Variables to store routes for team prop center for all four start locations
    public Action redAudienceBotTeamPropCenterRoute;
    public Action redBackstageBotTeamPropCenterRoute;
    public Action blueBackstageBotTeamPropCenterRoute;
    public Action blueAudienceBotTeamPropCenterRoute;

    //Variables to store routes for team prop left for all four start locations
    public Action redBackstageBotTeamPropLeftRoute;
    public Action blueAudienceBotTeamPropLeftRoute;
    public Action redAudienceBotTeamPropLeftRoute;
    public Action blueBackstageBotTeamPropLeftRoute;

    //Variables to store routes for team prop right for all four start locations
    public Action redBackstageBotTeamPropRightRoute;
    public Action redAudienceBotTeamPropRightRoute;
    public Action blueBackstageBotTeamPropRightRoute;
    public Action blueAudienceBotTeamPropRightRoute;

    public void BuildRoutes() {
        /** BLUE BACKSTAGE LEFT / RED BACKSTAGE RIGHT **/
        blueBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_L, TANGENT_315_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKSTAGE_START_LANE_A, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_R, TANGENT_45_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKSTAGE_START_LANE_F, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE RIGHT / RED BACKSTAGE LEFT **/
        blueBackstageBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_R, TANGENT_225_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_RIGHT, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                .build();

        redBackstageBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_L, TANGENT_135_DEGREES)
                .setReversed(true)
                .splineToLinearHeading(RED_BACKDROP_LEFT, TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE BACKSTAGE CENTER / RED BACKSTAGE CENTER **/
        blueBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_BACKSTAGE_START_POSE)
                .splineToLinearHeading(BLUE_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_BACKDROP_CENTER, FACE_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_A))
                                .build();

        redBackstageBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_BACKSTAGE_START_POSE)
                .splineToLinearHeading(RED_BACKSTAGE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(  RED_BACKDROP_CENTER.position.x+3,
                        RED_BACKDROP_CENTER.position.y,
                        FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .lineToX(RED_BACKDROP_CENTER.position.x-5.5)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_F))
                .build();

        /** BLUE AUDIENCE LEFT / RED AUDIENCE RIGHT **/
        blueAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_L, TANGENT_315_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_R), TANGENT_TOWARD_RED)
                .splineToLinearHeading(BLUE_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropRightRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_R, TANGENT_45_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_L), TANGENT_TOWARD_BLUE)
                .splineToLinearHeading(RED_STAGEDOOR_ENTRANCE, TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        /** BLUE AUDIENCE RIGHT / RED AUDIENCE LEFT **/
        Action blueAudienceBotTeamPropRightRouteA = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_R, FACE_225_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(BLUE_AUDIENCE_SPIKE_L), TANGENT_TOWARD_RED)
                .splineToLinearHeading(new Pose2d(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), FACE_TOWARD_BACKSTAGE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_RIGHT), TANGENT_TOWARD_BACKSTAGE)
                .build();

        Action blueAudienceBotTeamPropRightRouteB = roadRunnerDrive.actionBuilder(BLUE_BACKDROP_RIGHT)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        blueAudienceBotTeamPropRightRoute = new SequentialAction(
                blueAudienceBotTeamPropRightRouteA,
                blueAudienceBotTeamPropRightRouteB
        );

        redAudienceBotTeamPropLeftRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_L, FACE_135_DEGREES)
                .setReversed(true)
                .splineToConstantHeading(PoseToVector(RED_AUDIENCE_SPIKE_C), FACE_TOWARD_BLUE)
                .splineToLinearHeading(new Pose2d(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE), FACE_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_LEFT), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

        /** BLUE AUDIENCE CENTER / RED AUDIENCE CENTER **/
        blueAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(BLUE_AUDIENCE_START_POSE)
                .splineToLinearHeading(BLUE_AUDIENCE_SPIKE_C, TANGENT_TOWARD_RED)
                .setReversed(true)
                .splineToLinearHeading(BLUE_SAFE_STRAFE, TANGENT_TOWARD_RED)
                .splineToConstantHeading(PoseToVector(BLUE_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(BLUE_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(BLUE_BACKSTAGE_PARK_LANE_C))
                .turnTo(FACE_45_DEGREES)
                .build();

        redAudienceBotTeamPropCenterRoute = roadRunnerDrive.actionBuilder(RED_AUDIENCE_START_POSE)
                .splineToLinearHeading(RED_AUDIENCE_SPIKE_C, TANGENT_TOWARD_BLUE)
                .setReversed(true)
                .splineToLinearHeading(RED_SAFE_STRAFE, TANGENT_TOWARD_BLUE)
                .splineToConstantHeading(PoseToVector(RED_NEUTRAL_PIXEL_STAGEDOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_STAGEDOOR_ENTRANCE), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_THROUGH_DOOR), TANGENT_TOWARD_BACKSTAGE)
                .splineToConstantHeading(PoseToVector(RED_BACKDROP_CENTER), TANGENT_TOWARD_BACKSTAGE)
                .strafeTo(PoseToVector(RED_BACKSTAGE_PARK_LANE_D))
                .turnTo(FACE_315_DEGREES)
                .build();

    }

}
