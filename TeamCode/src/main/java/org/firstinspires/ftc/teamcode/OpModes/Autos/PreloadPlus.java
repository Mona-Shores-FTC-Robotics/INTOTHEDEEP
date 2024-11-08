package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.NET.LongSidePickup.NET_Score_2_Preload_and_1_Sample;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.PushAllAtOnce.OBS_Push3SpikeSampleInOnePath;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike;
import com.example.sharedconstants.Routes.Routes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
@Autonomous(name = "Preload Plus")
public class PreloadPlus extends LinearOpMode {
    Routes blueObsRoute;
    Routes blueNetRoute;
    Routes redObsRoute;
    Routes redNetRoute;

    @Override
    public void runOpMode() {
        Robot.createInstance(this);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        //Instantiate the robotDriveAdapter so we can use MeepMeep seamlessly
        RealRobotAdapter robotAdapter = new RealRobotAdapter();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        //Make the blue routes
        robotAdapter.setAllianceColor(FieldConstants.AllianceColor.BLUE);
        blueObsRoute = new OBS_Push3SpikeSampleInOnePath(robotAdapter);
        blueObsRoute.buildRoute();
        blueNetRoute = new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(robotAdapter);
        blueNetRoute.buildRoute();

        //Make the red routes
        robotAdapter.setAllianceColor(FieldConstants.AllianceColor.RED);
        redObsRoute = new OBS_Push3SpikeSampleInOnePath(robotAdapter);
        redObsRoute.buildRoute();
        redNetRoute = new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(robotAdapter);
        redNetRoute.buildRoute();

        while (opModeInInit()) {
            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.SelectAndLockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //set the color of our adapter to the final alliance color - this technically shouldn't matter
        robotAdapter.setAllianceColor(MatchConfig.finalAllianceColor);

        //Select route depending on side of field and color of alliance
        Routes route = selectRoute();

        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRoute = route.getRouteAction(MatchConfig.finalSideOfField);

        //set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose= FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        //Calculate the Yaw offset based on the starting pose and save it in MatchConfig to use during teleop field oriented control
        Robot.getInstance().getDriveSubsystem().CalculateYawOffset();

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(selectedRoute);

        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }

    public Routes selectRoute() {
        // Select route depending on side of field and alliance color
        Routes route;

        if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.NET) {
            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                // Case 1: Blue alliance on the NET side
                route = blueNetRoute;
            } else {
                // Case 2: Red alliance on the NET side
                route = redNetRoute;
            }
        } else {
            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                // Case 3: Blue alliance on the OBS side
                route = blueObsRoute;
            } else {
                // Case 4: Red alliance on the OBS side
                route = redObsRoute;
            }
        }
        return route;
    }
}

