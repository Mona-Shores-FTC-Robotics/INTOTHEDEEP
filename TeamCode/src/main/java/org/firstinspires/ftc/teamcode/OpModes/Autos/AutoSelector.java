package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.finalAllianceColor;
import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.finalSideOfField;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.SubsystemType.SPECIMEN_INTAKE;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.NET.NET_Score5_SamplePreload;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_2_Preload_and_1_Sample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_3_Preload_and_2_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_4_Preload_and_3_Samples_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short;
import com.example.sharedconstants.Routes.NET.ShortSidePickup.NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short;
import com.example.sharedconstants.Routes.OBS.InakeAndScore.OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike_Not_At_1_Time;
import com.example.sharedconstants.Routes.OBS.OBS_Intake_Automatic_Sample_Handling;
import com.example.sharedconstants.Routes.OBS.OBS_Intake_Transfer_Dump;
import com.example.sharedconstants.Routes.Routes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.ActionsAndCommands.AutonomousPeriodicAction;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "Auto Selector")
public class AutoSelector extends LinearOpMode {
    private RealRobotAdapter robotAdapter;
    private GamepadHandling gamepadHandling;
    private List<Routes> blueNetRoutesList = new ArrayList<>();
    private List<Routes> blueObsRoutesList = new ArrayList<>();
    private List<Routes> redNetRoutesList = new ArrayList<>();
    private List<Routes> redObsRoutesList = new ArrayList<>();
    private List<Routes> routeList;  // Holds the current list based on side of field
    private int selectedIndex = 0;
    private Routes selectedRoute;

    private List<Routes> buildNetRoutes(FieldConstants.AllianceColor allianceColor) {
        Routes netRoute;
        List<Routes> netRouteList = new ArrayList<>();
        robotAdapter.setAllianceColor(allianceColor);

        // Less points but scores perectly fine.
        netRoute = new NET_Score_2_Preload_and_1_Sample_Short(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // Score more points than last.
        netRoute = new NET_Score_3_Preload_and_2_Samples_Short(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // Same as the last one but Scores more points.
        netRoute = new NET_Score_4_Preload_and_3_Samples_Short(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute); // Tristan + Landon likes

        // In the risk of getting bumped by the other robot but scores more points.
        netRoute = new NET_Score_5_Preload_and_3_Samples_and_1_HumanPlayerSample_Short(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute); // Tristan + Landon likes

        // Scores more points but it might get more interfere with the robot because it goes to the observation zone twice.
        netRoute = new NET_Score_6_Preload_and_3_Samples_and_2_HumanPlayerSamples_Short(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // Scores less points.
        netRoute = new NET_Score_1_Specimen_Preload(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // It scores a lot of points and it won't inter fear with the robot.
        netRoute = new NET_Score5_SamplePreload(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute); // Tristan + Landon likes

        return netRouteList;
    }

    private List<Routes> buildOBSRoutes(FieldConstants.AllianceColor allianceColor) {
        Routes obsRoute;
        List<Routes> obsRouteList = new ArrayList<>();
        robotAdapter.setAllianceColor(allianceColor);

        obsRoute = new OBS_Intake_Automatic_Sample_Handling(robotAdapter);
        obsRoute.buildRoute();
        obsRouteList.add(obsRoute);

        // Gets in the way of the other side also tramples over samples.
        obsRoute = new OBS_Intake_Transfer_Dump(robotAdapter);
        obsRoute.buildRoute();
        obsRouteList.add(obsRoute); // Tristan + Landon likes

        obsRoute = new OBS_Intake_3_Score_4_Specimens_Preload_And_1_Premade_And_3_Spike_Not_At_1_Time(robotAdapter);
        obsRoute.buildRoute();
        obsRouteList.add(obsRoute);

        return obsRouteList;
    }

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler
        CommandScheduler.getInstance().reset();

        // Create and Initialize the robot
        Robot.createInstance(this);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        // Initialize the Gamepad Handling
        gamepadHandling = new GamepadHandling(this);

        // Create the RealRobotAdapter instance
        robotAdapter = new RealRobotAdapter();

        buildRoutes();

        // Perform route selection during init
        while (opModeInInit()) {
            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();

            // Monitor preload: respects locked state
            if (Robot.getInstance().hasSubsystem(SPECIMEN_INTAKE)) {
                Robot.getInstance()
                        .getSpecimenIntakeSubsystem()
                        .monitorSpecimenPreload(robotAdapter , gamepadHandling.LockedSettingsFlag , gamepadHandling.manualOverrideFlag);
            }

            // Allow driver to override and lock alliance color and side
            gamepadHandling.SelectAllianceAndSide(telemetry);

            //Handle Lighting During Init
            if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.LIGHTING)) {
                Robot.getInstance().getLightingSubsystem().updateLightsBasedOnAllianceColorAndSide(finalAllianceColor , finalSideOfField);
            }

            telemetry.update();
            sleep(10);
        }
        runSelectedRoute();
    }

    private void buildRoutes() {
        // Disable auto-clear to keep telemetry messages on the screen
        telemetry.setAutoClear(false);
        telemetry.addLine("Starting to build routes...");
        telemetry.update();
        //This builds routes by temporarily changing our color to blue/red, then storing them in their own lists so that we can use them during init
        blueObsRoutesList = buildOBSRoutes(FieldConstants.AllianceColor.BLUE);
        telemetry.addLine("Blue OBS route build complete.");
        telemetry.update();
        blueNetRoutesList = buildNetRoutes(FieldConstants.AllianceColor.BLUE);
        telemetry.addLine("Blue NET route build complete.");
        telemetry.update();
        redObsRoutesList = buildOBSRoutes(FieldConstants.AllianceColor.RED);
        telemetry.addLine("Red OBS route build complete.");
        telemetry.update();
        redNetRoutesList = buildNetRoutes(FieldConstants.AllianceColor.RED);
        telemetry.addLine("Red NET route build complete.");
        telemetry.update();
        // Re-enable auto-clear to return to the default behavior
        sleep(1000);
        telemetry.clearAll();
        telemetry.setAutoClear(true);
    }

    private void selectRoute() {
        // Assign routeList based on side of the field / alliance color
        if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.NET && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            if (routeList != blueNetRoutesList) {
                routeList = blueNetRoutesList;
                selectedIndex = 0;  // Reset index on side change
            }
        } else if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.OBSERVATION && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            if (routeList != blueObsRoutesList) {
                routeList = blueObsRoutesList;
                selectedIndex = 0;  // Reset index on side change
            }
        } else if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.NET && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) {
            if (routeList != redNetRoutesList) {
                routeList = redNetRoutesList;
                selectedIndex = 0;  // Reset index on side change
            }
        } else if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.OBSERVATION && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) {
            if (routeList != redObsRoutesList) {
                routeList = redObsRoutesList;
                selectedIndex = 0;  // Reset index on side change
            }
        }
        //set the color of our adapter to the final alliance color - this technically shouldn't matter
        robotAdapter.setAllianceColor(MatchConfig.finalAllianceColor);

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
    }

    private void runSelectedRoute() {
        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRouteAction = selectedRoute.getRouteAction(MatchConfig.finalSideOfField);

        //Set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        //Calculate the Yaw offset based on the starting pose and save it in MatchConfig
        Robot.getInstance().getDriveSubsystem().CalculateYawOffset();

        telemetry.clearAll();

        AutonomousPeriodicAction autonomousPeriodicAction = new AutonomousPeriodicAction();
        MatchConfig.telemetryPacket = new TelemetryPacket();
        ParallelAction parallelAction = new ParallelAction(
                selectedRouteAction,
                autonomousPeriodicAction
        );

        MatchConfig.loopTimer = new ElapsedTime();
        MatchConfig.loopTimer.reset();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(parallelAction);

        // Update final autonomous data

        //TODO can we set a flag so we know that auto was run when we get to teleop and then we can handle things differently.
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}
