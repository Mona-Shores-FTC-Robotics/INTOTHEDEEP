package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import com.example.sharedconstants.Routes.Routes;
import org.reflections.Reflections;
import org.reflections.scanners.SubTypesScanner;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

@Autonomous(name = "Auto Select Op Mode")
public class AutoSelectionOpMode extends LinearOpMode {
    private RealRobotAdapter robotDriveAdapter;
    private GamepadHandling gamepadHandling;
    private List<Class<? extends Routes>> availableRoutes;
    private List<Class<? extends Routes>> filteredRoutes;
    private int selectedIndex = 0;
    private Routes selectedRoute;
    private FieldConstants.SideOfField previousSideOfField = null;

    @Override
    public void runOpMode() {
        // Reset the Singleton CommandScheduler
        CommandScheduler.getInstance().reset();

        // Initialize the Gamepad Handling
        gamepadHandling = new GamepadHandling(this);

        // Find available routes dynamically using Reflections
        Reflections reflections = new Reflections("com.example.sharedconstants.Routes", new SubTypesScanner(false));
        Set<Class<? extends Routes>> routesSet = reflections.getSubTypesOf(Routes.class);
        availableRoutes = new ArrayList<>(routesSet);

        // Sort the routes alphabetically for easier navigation
        availableRoutes.sort((r1, r2) -> r1.getSimpleName().compareToIgnoreCase(r2.getSimpleName()));

        while (opModeInInit()) {
            // Allow driver to override/lock the vision and select settings
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.SelectAndLockColorAndSideAndRobotType(telemetry);

            // Update filtered routes if side of field changes
            updateFilteredRoutesIfSideChanged();

            // Use the gamepadHandling method to cycle through filtered routes
            if (filteredRoutes != null && !filteredRoutes.isEmpty()) {
                selectedIndex = gamepadHandling.cycleThroughRoutes(filteredRoutes, selectedIndex);

                // Display the selected route on the driver station
                telemetry.addData("Selected Auto Route", filteredRoutes.get(selectedIndex).getSimpleName());
            }

            telemetry.update();
            sleep(50);  // Prevent overloading the loop
        }

        // Lock in the selected route
        try {
            robotDriveAdapter = new RealRobotAdapter();
            selectedRoute = filteredRoutes.get(selectedIndex).getConstructor(RealRobotAdapter.class).newInstance(robotDriveAdapter);
            selectedRoute.buildRoute();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to instantiate selected route.");
            telemetry.update();
            return;
        }

        // Create and Initialize the robot
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        // Set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField,MatchConfig.finalAllianceColor);

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        // Run the selected route
        Actions.runBlocking(selectedRoute.getRouteAction(MatchConfig.finalSideOfField));

        // Update final autonomous data
        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }

    private void updateFilteredRoutesIfSideChanged() {
        if (MatchConfig.finalSideOfField != null && !MatchConfig.finalSideOfField.equals(previousSideOfField)) {
            // Update the previous side of field
            previousSideOfField = MatchConfig.finalSideOfField;

            // Filter routes based on the selected side of the field
            String sidePrefix = MatchConfig.finalSideOfField == FieldConstants.SideOfField.OBSERVATION ? "OBS_" : "NET_";
            filteredRoutes = new ArrayList<>();
            for (Class<? extends Routes> route : availableRoutes) {
                if (route.getSimpleName().startsWith(sidePrefix)) {
                    filteredRoutes.add(route);
                }
            }
            selectedIndex = 0; // Reset selected index when filtering changes
        }
    }
}
