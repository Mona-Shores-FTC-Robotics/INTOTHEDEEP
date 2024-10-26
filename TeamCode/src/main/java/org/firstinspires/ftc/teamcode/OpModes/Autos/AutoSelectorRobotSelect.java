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
import java.util.List;

@Autonomous(name = "Auto Selector [Robot Select]")
public class AutoSelectorRobotSelect extends LinearOpMode {

    private RealRobotAdapter adapter;
    private GamepadHandling gamepadHandling;
    private List<Routes> netRoutesList = new ArrayList<>();
    private List<Routes> obsRoutesList = new ArrayList<>();
    private List<Routes> routeList;  // Holds the current list based on side of field
    private int selectedIndex = 0;
    private Routes selectedRoute;

    @Override
    public void runOpMode() {
        // Reset the Singleton CommandScheduler
        CommandScheduler.getInstance().reset();

        // Initialize the Gamepad Handling
        gamepadHandling = new GamepadHandling(this);

        // Create and Initialize a dummy robot since we dont know the real robot type
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Create a dummy adapter so we can build the routes
        adapter = new RealRobotAdapter();

        buildRoutes();
        // Perform route selection during init
        while (opModeInInit()) {
            initRouteSelection();
        }
        //re initialize now that robot is set right
        initializeComponents();
        runSelectedRoute();
    }

    private void initializeComponents() {
        // Create and Initialize the robot
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        // Set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        // Create the RealRobotAdapter instance
        adapter = new RealRobotAdapter();
    }

    private void buildRoutes() {
        // OBS-specific routes with descriptive names
        Routes route = new OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Score_1_Specimen_Preload(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Score_4_SampleFirst_Push_2_Spike_Samples(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Score_5_SampleFirst_Push_3_Spike_Samples(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Push3SpikeSampleInOnePath(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        route = new OBS_Push2SpikeSamplesInOnePath(adapter);
        route.buildRoute();
        obsRoutesList.add(route);

        // NET-specific routes
        route = new NET_Score_2_Preload_and_1_Sample_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        route = new NET_Score_3_Preload_and_2_Samples_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        route = new NET_Score_4_Preload_and_3_Samples_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        route = new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        route = new NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        route = new NET_Score_1_Specimen_Preload(adapter);
        route.buildRoute();
        netRoutesList.add(route);

        route = new NET_Score5_SamplePreload(adapter);
        route.buildRoute();
        netRoutesList.add(route);

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
        gamepadHandling.SelectAndLockColorAndSideAndRobotType(telemetry);

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
