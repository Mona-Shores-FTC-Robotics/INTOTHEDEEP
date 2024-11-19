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
import com.example.sharedconstants.Routes.DoNothing;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_2_Preload_and_1_Sample;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_3_Preload_and_2_Samples;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_4_Preload_and_3_Samples;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.OBS_Score4_Preload_Push_Two_And_Pickup_At_Triangle;
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

        // Tested 11-15-24
        netRoute = new NET_Score_1_Specimen_Preload(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // Tested 11-15-24
        netRoute = new NET_Score_2_Preload_and_1_Sample(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // Tested 11-15-24
        netRoute = new NET_Score_3_Preload_and_2_Samples(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute);

        // Same as the last one but Scores more points.
        netRoute = new NET_Score_4_Preload_and_3_Samples(robotAdapter);
        netRoute.buildRoute();
        netRouteList.add(netRoute); // Tristan + Landon likes

        return netRouteList;
    }

    private List<Routes> buildOBSRoutes(FieldConstants.AllianceColor allianceColor) {
        Routes obsRoute;
        List<Routes> obsRouteList = new ArrayList<>();
        robotAdapter.setAllianceColor(allianceColor);

        //Best Auto but needs high speeds (~50 velocity/acceleration)
        //This would just leave a preload sample by our robot for the NET bot to score and then score all 5 specimens
//        obsRoute = new OBS_Score5_Leave_Preload_Push_All_And_Pickup_At_Triangle(robotAdapter);
//        obsRoute.buildRoute();
//        obsRouteList.add(obsRoute);

        //Superb auto, but needs high speeds (~50 velocity/acceleration)
        //Scores preload Specimen, pushes 3 spikes to Human player, and then picks four specimens up from human player and scores them
//        obsRoute = new OBS_Score5_Preload_Push_All_And_Pickup_At_Triangle(robotAdapter);
//        obsRoute.buildRoute();
//        obsRouteList.add(obsRoute);

        //Most Reasonable Auto (~25 velocity/acceleration)
        //Scores preload Specimen, pushes 2 spikes to Human player, and then picks three specimens up from human player and scores them
        //This assumes our partner did NOT take one of the Human Player starting specimens
        obsRoute = new OBS_Score4_Preload_Push_Two_And_Pickup_At_Triangle(robotAdapter);
        obsRoute.buildRoute();
        obsRouteList.add(obsRoute);

        //Next Most Reasonable Auto (~30 velocity/acceleration)
        //Scores preload Specimen, pushes 3 spikes to Human player, and then picks three specimens up from human player and scores them
        //This assumes our partner took one of the Human Player starting specimens
//        obsRoute = new OBS_Score4_Preload_Push_All_And_Pickup_At_Triangle(robotAdapter);
//        obsRoute.buildRoute();
//        obsRouteList.add(obsRoute);

        obsRoute = new OBS_Score_1_Specimen_Preload(robotAdapter);
        obsRoute.buildRoute();
        obsRouteList.add(obsRoute);

        obsRoute = new DoNothing(robotAdapter);
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
            selectRoute();


            //Handle Lighting During Init
            if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.LIGHTING)) {
                Robot.getInstance().getLightingSubsystem().updateLightBasedOnPreloadPresenceAndAllianceColorAndSideOfField(finalAllianceColor , finalSideOfField);
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
        MatchConfig.hasAutoRun=true;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}
