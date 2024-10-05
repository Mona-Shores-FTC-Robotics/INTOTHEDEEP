package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.OBS_Preload_and_One_Specimen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Autonomous(name = "Preload Auto Plus One Speciman")
public class PreloadPlusOneSpecimanAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        while (opModeInInit()) {
            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.SelectAndLockColorAndSideAndRobotType(telemetry);
            telemetry.update();
            sleep(10);
        }

        //TODO: before competition we must hardcode for a single robot and move this to before the init to save precious auto time

        // Create and Initialize the robot
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        //Instantiate the robotDriveAdapter so we can use MeepMeep seamlessly
        RealRobotAdapter robotDriveAdapter = new RealRobotAdapter();

        //Build all the routes using the adapter so we can select one quickly later
        OBS_Preload_and_One_Specimen preloadAndOneSpecimen = new OBS_Preload_and_One_Specimen(robotDriveAdapter);
        preloadAndOneSpecimen.BuildRoutes();


        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRoute = preloadAndOneSpecimen.getRouteAction(MatchConfig.finalSideOfField);

        //set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose= FieldConstants.getStartPose(MatchConfig.finalAllianceColor, MatchConfig.finalSideOfField);

        //Reset Gyro
        //TODO i suspect this is not needed or might be duplicative with what is happening in MecanumDrive
        // can we run a test without this and see what happens?
        //Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(selectedRoute);

        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffset;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}

