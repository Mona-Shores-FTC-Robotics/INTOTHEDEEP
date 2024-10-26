package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
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
import java.util.Set;

@Autonomous(name = "Observation Autos Op Mode")
public class ObservationAutos extends LinearOpMode {

    private RealRobotAdapter robotDriveAdapter;
    private GamepadHandling gamepadHandling;
    private List<Routes> availableRoutes;
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
        robotDriveAdapter = new RealRobotAdapter();
    }

    private void buildRoutes() {

    }

    private void initRouteSelection() {
        // Use the gamepadHandling method to cycle through available routes without side selection during init
            gamepadHandling.getDriverGamepad().readButtons();
            selectedIndex = gamepadHandling.cycleThroughRoutes2(availableRoutes, selectedIndex);
            gamepadHandling.SelectAndLockColor();

            // Display the selected route on the driver station
            telemetry.addLine();
            telemetry.addLine("Use Operator D-PAD LEFT/RIGHT to cycle through OBSERVATION Auto Routes");
            telemetry.addData("Selected Observation Auto Route: ", availableRoutes.get(selectedIndex).getClass().getSimpleName());
            telemetry.update();
            sleep(50);  // Prevent overloading the loop
    }

    private void runSelectedRoute() {
        // Lock in the selected route
        selectedRoute = availableRoutes.get(selectedIndex);
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
