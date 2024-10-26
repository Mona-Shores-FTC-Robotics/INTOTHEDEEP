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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Autonomous(name = "Observation Autos Op Mode")
public class AutoSelector extends LinearOpMode {

    private RealRobotAdapter adapter;
    private GamepadHandling gamepadHandling;
    private HashMap<String, Routes> obsRoutesMap;
    private HashMap<String, Routes> netRoutesMap;

    private int selectedIndex = 0;
    private Routes selectedRoute;

    private Map<Routes, Map<FieldConstants.AllianceColor, Action>> routeActionsMap = new HashMap<>();

    @Override
    public void runOpMode() {
        // Hardcode the Robot Type and Side of Field
        MatchConfig.finalRobotType = Robot.RobotType.CHASSIS_19429_B_PINPOINT;
        MatchConfig.finalSideOfField = FieldConstants.SideOfField.OBSERVATION;

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

        // Set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        // Create the RealRobotAdapter instance
        adapter = new RealRobotAdapter();
    }

    private void buildRoutes() {

// NET-specific routes
        // HashMap for OBS-specific routes with descriptive names
        obsRoutesMap = new HashMap<>();
        obsRoutesMap.put("OBS Push 2 Score 3 Specimens Preload And 1 Premade And 1 Spike", new OBS_Push_2_Score_3_Specimens_Preload_And_1_Premade_And_1_Spike(adapter));
        obsRoutesMap.put("OBS Push 3 Score 4 Specimens Preload And 1 Premade And 2 Spike", new OBS_Push_3_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter));
        obsRoutesMap.put("OBS Push 3 Score 5 Specimens Preload And 1 Premade And 3 Spike", new OBS_Push_3_Score_5_Specimens_Preload_And_1_Premade_And_3_Spike(adapter));
        obsRoutesMap.put("OBS Push 2 Score 4 Specimens Preload And 1 Premade And 2 Spike", new OBS_Push_2_Score_4_Specimens_Preload_And_1_Premade_And_2_Spike(adapter));
        obsRoutesMap.put("OBS Score 1 Sample Preload Push 1 Spike Score 1 Premade", new OBS_Score_1_Sample_Preload_Push_1_Spike_Score_1_Premade(adapter));
        obsRoutesMap.put("OBS Score 1 Preload", new OBS_Score_1_Specimen_Preload(adapter));
        obsRoutesMap.put("OBS Score 4 SampleFirst Push 2 Spike Samples", new OBS_Score_4_SampleFirst_Push_2_Spike_Samples(adapter));
        obsRoutesMap.put("OBS Score 5 SampleFirst Push 3 Spike Samples", new OBS_Score_5_SampleFirst_Push_3_Spike_Samples(adapter));
        obsRoutesMap.put("OBS Push 3 Spike Samples in One Path", new OBS_Push3SpikeSampleInOnePath(adapter));
        obsRoutesMap.put("OBS Push 2 Spike Samples in One Path", new OBS_Push2SpikeSamplesInOnePath(adapter));
        obsRoutesMap.put("Move Only", new MoveOnly(adapter));    // Shared option
        obsRoutesMap.put("Do Nothing", new DoNothing(adapter));  // Shared option

        netRoutesMap = new HashMap<>();
        netRoutesMap.put("NET Score 2 Preload and 1 Sample Short", new NET_Score_2_Preload_and_1_Sample_Short(adapter));
        netRoutesMap.put("NET Score 3 Preload and 2 Samples Short", new NET_Score_3_Preload_and_2_Samples_Short(adapter));
        netRoutesMap.put("NET Score 4 Preload and 3 Samples Short", new NET_Score_4_Preload_and_3_Samples_Short(adapter));
        netRoutesMap.put("NET Score 5 Preload and 3 Samples and 1 Human Player Sample Short", new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(adapter));
        netRoutesMap.put("NET Score 6 Preload and 3 Samples and 2 Human Player Samples Short", new NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short(adapter));
        netRoutesMap.put("NET Score 1 Preload", new NET_Score_1_Specimen_Preload(adapter));
        netRoutesMap.put("NET Score 5 Sample Preload", new NET_Score5_SamplePreload(adapter));
        netRoutesMap.put("Move Only", new MoveOnly(adapter));    // Shared option
        netRoutesMap.put("Do Nothing", new DoNothing(adapter));  // Shared option

    }

    private void initRouteSelection() {
        gamepadHandling.getDriverGamepad().readButtons();
        gamepadHandling.SelectAndLockColorAndSideAndRobotType(telemetry);

        if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.NET) {
            selectedIndex = gamepadHandling.cycleThroughRoutes(netRoutesMap, selectedIndex);
            String selectedRouteName = new ArrayList<>(netRoutesMap.keySet()).get(selectedIndex);
            selectedRoute = netRoutesMap.get(selectedRouteName);
        } else {
            selectedIndex = gamepadHandling.cycleThroughRoutes(obsRoutesMap, selectedIndex);
            String selectedRouteName = new ArrayList<>(obsRoutesMap.keySet()).get(selectedIndex);
            selectedRoute = obsRoutesMap.get(selectedRouteName);
        }

        telemetry.addData("Selected Route", selectedRoute);
        telemetry.update();
        sleep(50);
    }

    private void runSelectedRoute() {
        // Lock in the selected route
        Action selectedRouteAction = routeActionsMap.get(selectedRoute).get(MatchConfig.finalAllianceColor);

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        // Run the selected route
        if (selectedRouteAction != null) {
            Actions.runBlocking(selectedRouteAction);
        }

        // Update final autonomous data
        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}
