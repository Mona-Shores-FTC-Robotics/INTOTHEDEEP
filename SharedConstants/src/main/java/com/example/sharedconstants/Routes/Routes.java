package com.example.sharedconstants.Routes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.RobotAdapter;

import java.util.Arrays;

public abstract class Routes {

    // Velocity and acceleration overrides
    public static final double SLOW_VELOCITY_OVERRIDE = 10;
    public static final double SLOW_ACCELERATION_OVERRIDE = 15;
    public static final double SLOW_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(45);

    public static final double NORMAL_VELOCITY_OVERRIDE = 40;
    public static final double NORMAL_ACCELERATION_OVERRIDE = 40;
    public static final double NORMAL_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(90);

    public static final double FAST_VELOCITY_OVERRIDE = 60;
    public static final double FAST_ACCELERATION_OVERRIDE = 60;
    public static final double FAST_ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(120);

    // Shared constraints for all routes
    public static VelConstraint slowVelocity;
    public static AccelConstraint slowAcceleration;
    public static VelConstraint normalVelocity;
    public static AccelConstraint normalAcceleration;
    public static VelConstraint fastVelocity;
    public static AccelConstraint fastAcceleration;

    protected RobotAdapter robotAdapter;

    public Routes(RobotAdapter robotAdapter) {
        this.robotAdapter = robotAdapter;
        setupConstraints();
    }

    public abstract void BuildRoutes();

    // Variables to store routes for all start locations and team prop locations
    protected Action redAudienceBotRoute;
    protected Action redBackstageBotRoute;
    protected Action blueBackstageBotRoute;
    protected Action blueAudienceBotRoute;

    // These are the defaults
    //THIS SAYS RED AUDIENCE BECXAUSE IT IS MIRRORED
    protected Pose2d redAudienceStartPose = RED_AUDIENCE_START_POSE;
    protected Pose2d blueBackstageStartPose = BLUE_BACKSTAGE_START_POSE;

    //THIS SAYS RED BACKSTAGE BECAUSE ITS MIRRORED
    protected Pose2d redBackstageStartPose = RED_BACKSTAGE_START_POSE;
    protected Pose2d blueAudienceStartPose = BLUE_AUDIENCE_START_POSE;

    // Method to return the corresponding starting pose
    public Pose2d getStartingPose(AllianceColor allianceColor, SideOfField sideOfField) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        return blueAudienceStartPose;
                    case BACKSTAGE:
                        return blueBackstageStartPose;
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        return redAudienceStartPose;
                    case BACKSTAGE:
                        return redBackstageStartPose;
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    public Action getRoute(AllianceColor allianceColor, SideOfField sideOfField) {
        switch (allianceColor) {
            case BLUE:
                switch (sideOfField) {
                    case AUDIENCE:
                        return getBlueAudienceBotRoute();  // Consolidated method for BLUE AUDIENCE
                    case BACKSTAGE:
                        return getBlueBackstageBotRoute();  // Consolidated method for BLUE BACKSTAGE
                }
                break;
            case RED:
                switch (sideOfField) {
                    case AUDIENCE:
                        return getRedAudienceBotRoute();  // Consolidated method for RED AUDIENCE
                    case BACKSTAGE:
                        return getRedBackstageBotRoute();  // Consolidated method for RED BACKSTAGE
                }
                break;
        }
        return null;  // Fallback in case nothing matches
    }

    public Action getRedAudienceBotRoute() {
        return redAudienceBotRoute;
    }

    public Action getBlueAudienceBotRoute() {
        return blueAudienceBotRoute;
    }

    public Action getBlueBackstageBotRoute() {
        return blueBackstageBotRoute;
    }

    public Action getRedBackstageBotRoute() {
        return redBackstageBotRoute;
    }

    private void setupConstraints() {
        slowVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SLOW_VELOCITY_OVERRIDE),
                new AngularVelConstraint(SLOW_ANGULAR_VELOCITY_OVERRIDE)
        ));
        slowAcceleration = new ProfileAccelConstraint(-SLOW_ACCELERATION_OVERRIDE, SLOW_ACCELERATION_OVERRIDE);

        normalVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(NORMAL_VELOCITY_OVERRIDE),
                new AngularVelConstraint(NORMAL_ANGULAR_VELOCITY_OVERRIDE)
        ));
        normalAcceleration = new ProfileAccelConstraint(-NORMAL_ACCELERATION_OVERRIDE, NORMAL_ACCELERATION_OVERRIDE);

        fastVelocity = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(FAST_VELOCITY_OVERRIDE),
                new AngularVelConstraint(FAST_ANGULAR_VELOCITY_OVERRIDE)
        ));
        fastAcceleration = new ProfileAccelConstraint(-FAST_ACCELERATION_OVERRIDE, FAST_ACCELERATION_OVERRIDE);
    }
}