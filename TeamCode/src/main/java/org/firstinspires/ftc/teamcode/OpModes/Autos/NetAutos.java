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
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ConfigurationBuilder;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;


@Autonomous(name = "Net Autos Op Mode")
public class NetAutos extends LinearOpMode {

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
        MatchConfig.finalSideOfField = FieldConstants.SideOfField.NET;

        initializeComponents();

        buildRoutes();


        // Perform route selection during init
        while (opModeInInit()) {
            initRouteSelection();
        }
        runSelectedRoute();
    }

    private void initializeComponents() {
        CommandScheduler.getInstance().reset();
        gamepadHandling = new GamepadHandling(this);

        Robot.createInstance(this, MatchConfig.finalRobotType);
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose =
                FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        robotDriveAdapter = new RealRobotAdapter();
    }

    private void buildRoutes() {

        String packageName = "org.firstinspires.ftc.teamcode.OpModes.Autos";

        Reflections reflections = new Reflections(new ConfigurationBuilder()
                .forPackages(packageName)
                .setScanners(Scanners.TypesAnnotated, Scanners.SubTypes));

        Set<Class<?>> classes = reflections.getSubTypesOf(Object.class);

        if (classes.isEmpty()) {
            System.out.println("No classes found in package: " + packageName);
        } else {
            for (Class<?> cls : classes) {
                System.out.println(cls.getName());
            }
        }

    }

    private void buildAndStoreActionsForRoute(Routes route) {
        try {
            robotDriveAdapter.setAllianceColor(FieldConstants.AllianceColor.RED);
            route.buildRoute();
            Action redRouteAction = route.getNetBotRoute();

            robotDriveAdapter.setAllianceColor(FieldConstants.AllianceColor.BLUE);
            route.buildRoute();
            Action blueRouteAction = route.getNetBotRoute();

            Map<FieldConstants.AllianceColor, Action> actionsMap = new HashMap<>();
            actionsMap.put(FieldConstants.AllianceColor.RED, redRouteAction);
            actionsMap.put(FieldConstants.AllianceColor.BLUE, blueRouteAction);
            routeActionsMap.put(route, actionsMap);

        } catch (Exception e) {
            telemetry.addData("Error", "Failed to build actions for route: " + route.getClass().getSimpleName());
        }
    }

    private void initRouteSelection() {
        gamepadHandling.getDriverGamepad().readButtons();
        selectedIndex = gamepadHandling.cycleThroughRoutes2(availableRoutes, selectedIndex);
        gamepadHandling.SelectAndLockColor();

        telemetry.addLine();
        telemetry.addLine("Use Operator D-PAD LEFT/RIGHT to cycle through NET Auto Routes");
        telemetry.addData("Selected Net Auto Route: ", availableRoutes.get(selectedIndex).getClass().getSimpleName());
        telemetry.update();
        sleep(50);
    }

    private void runSelectedRoute() {
        selectedRoute = availableRoutes.get(selectedIndex);
        Action selectedRouteAction = routeActionsMap.get(selectedRoute).get(MatchConfig.finalAllianceColor);

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        if (selectedRouteAction != null) {
            Actions.runBlocking(selectedRouteAction);
        }

        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}
