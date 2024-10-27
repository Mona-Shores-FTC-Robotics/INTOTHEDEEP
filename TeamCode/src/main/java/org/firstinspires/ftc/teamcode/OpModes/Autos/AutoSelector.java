package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.DoNothing;
import com.example.sharedconstants.Routes.MoveOnly;
import com.example.sharedconstants.Routes.NET.NET_Score5_SamplePreload;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_2_Preload_and_1_Sample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_3_Preload_and_2_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_4_Preload_and_3_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short;
import com.example.sharedconstants.Routes.OBS.InakeAndScore.OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike;
import com.example.sharedconstants.Routes.OBS.InakeAndScore.OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike_Not_At_1_Time;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.PushAllAtOnce.OBS_Push2SpikeSamplesInOnePath;
import com.example.sharedconstants.Routes.OBS.PushAllAtOnce.OBS_Push3SpikeSampleInOnePath;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike;
import com.example.sharedconstants.Routes.OBS.PushAndScore.OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike;
import com.example.sharedconstants.Routes.OBS.SampleFirst.OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade;
import com.example.sharedconstants.Routes.OBS.SampleFirst.OBS_Score_4_SampleFirst_Push_2_Spike_Samples;
import com.example.sharedconstants.Routes.OBS.SampleFirst.OBS_Score_5_SampleFirst_Push_3_Spike_Samples;
import com.example.sharedconstants.Routes.Routes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Autonomous(name = "Auto Selector Build Before Init")
public class AutoSelector extends LinearOpMode {

    private RealRobotAdapter adapter;
    private GamepadHandling gamepadHandling;
    private List<Routes> netRoutesList = new ArrayList<>();
    private List<Routes> obsRoutesList = new ArrayList<>();
    private List<Routes> routeList;  // Holds the current list based on side of field
    private int selectedIndex = 0;
    private Routes selectedRoute;

    @Override
    public void runOpMode() {
        // Defaults Robot, Side of Field, and Color
        // We will need to update the robot manually. its just too difficult to select the robot for autos.
        // Select robot is easy for teleop
        MatchConfig.finalRobotType = Robot.RobotType.CHASSIS_19429_B_PINPOINT;
        MatchConfig.finalSideOfField = FieldConstants.SideOfField.NET;
        MatchConfig.finalAllianceColor = FieldConstants.AllianceColor.BLUE;

        initializeComponents();
        buildRoutes();

        // Perform route selection during init
        while (opModeInInit()) {
            initRouteSelection();
        }
        runSelectedRoute();
    }

    private void initializeComponents() {
        // Reset the Singleton CommandScheduler
        CommandScheduler.getInstance().reset();

        // Initialize the Gamepad Handling
        gamepadHandling = new GamepadHandling(this);

        // Create and Initialize the robot
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        // Create the RealRobotAdapter instance
        adapter = new RealRobotAdapter();
    }

    private void buildRoutes() {
        // OBS-specific routes with descriptive names

        // Stays out of the other side and scores a lot of points / tramples over samples
        Routes route = new OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // It runs into the parts and tramples over them but if adjusted would score a lot / tramples over samples
        route = new OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // Gets in the way of the other side also tramples over samples.
        route = new OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route); // Tristan + Landon likes

        // Doesn't wait long enough to pick the specimen up.
        route = new OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route); // Tristan + Landon likes

        // Doesn't score enough points but could be used if the other team in our alliance is good.
        route = new OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // Scores less points.
        route = new OBS_Score_1_Specimen_Preload(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // Scored all samples but could possibly get rammed by the other robot.
        // Scores decent points.
        route = new OBS_Score_4_SampleFirst_Push_2_Spike_Samples(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // Scored all samples but could possibly get rammed by the other robot.
        // Scores high points.
        route = new OBS_Score_5_SampleFirst_Push_3_Spike_Samples(adapter);
        route.buildRoute();
        obsRoutesList.add(route); // Tristan + Landon likes

        // Doesn't push the samples whatsoever and did not score anything could also ram into the other alliances robots.
        route = new OBS_Push3SpikeSampleInOnePath(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // Pushes the samples perfect for when the other teams robot is also trying to score.
        route = new OBS_Push2SpikeSamplesInOnePath(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike_Not_At_1_Time(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);


        // NET-specific routes

        // Less points but scores perectly fine.
        route = new NET_Score_2_Preload_and_1_Sample_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        // Score more points than last.
        route = new NET_Score_3_Preload_and_2_Samples_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        // Same as the last one but Scores more points.
        route = new NET_Score_4_Preload_and_3_Samples_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route); // Tristan + Landon likes

        // In the risk of getting bumped by the other robot but scores more points.
        route = new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route); // Tristan + Landon likes

        // Scores more points but it might get more interfere with the robot because it goes to the observation zone twice.
        route = new NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        // Scores less points.
        route = new NET_Score_1_Specimen_Preload(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        // It scores a lot of points and it won't inter fear with the robot.
        route = new NET_Score5_SamplePreload(adapter);
        route.buildRoute();
        netRoutesList.add(route); // Tristan + Landon likes

        // Shared options (without buildRoute if not needed)
        Routes moveOnly = new MoveOnly(adapter);
        moveOnly.buildRoute();  // Optional, if needed
        obsRoutesList.add(moveOnly);
        netRoutesList.add(moveOnly);

        Routes doNothing = new DoNothing(adapter);
        doNothing.buildRoute();  // Optional, if needed
        obsRoutesList.add(doNothing);
        netRoutesList.add(doNothing);
    }

    private void initRouteSelection() {
        gamepadHandling.getDriverGamepad().readButtons();
        gamepadHandling.SelectAndLockColorAndSide();

        // Assign routeList based on side of the field, only once per side change
        if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.NET) {
            if (routeList != netRoutesList) {
                routeList = netRoutesList;
                selectedIndex = 0;  // Reset index on side change
            }
        } else {
            if (routeList != obsRoutesList) {
                routeList = obsRoutesList;
                selectedIndex = 0;  // Reset index on side change
            }
        }

        // Cycle through routes using gamepadHandling's method
        selectedIndex = gamepadHandling.cycleThroughRoutes(routeList, selectedIndex);

        // Update selected route based on current index
        selectedRoute = routeList.get(selectedIndex);

        // Display selected route on telemetry
        telemetry.addLine();
        telemetry.addData("Selected Index", selectedIndex);
        telemetry.addData("Total Routes", routeList.size());
        telemetry.addLine("Selected Route:");
        telemetry.addLine(selectedRoute.getClass().getSimpleName());

        telemetry.update();
        sleep(50);  // Small debounce delay
    }

    private void runSelectedRoute() {
        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRouteAction = selectedRoute.getRouteAction(MatchConfig.finalSideOfField);

//         Set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(selectedRouteAction);

        // Update final autonomous data
        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}
