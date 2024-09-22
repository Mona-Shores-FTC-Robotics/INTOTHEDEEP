package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.Routes.DirectRoutesExample;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.RealRobotAdapter;

@Autonomous(name = "Direct Routes Example Auto")
public class DirectRoutesExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        // Create and Initialize the robot
        Robot.createInstance(this, Robot.RobotType.ROBOT_CHASSIS);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        //Instantiate the robotDriveAdapter so we can use MeepMeep seamlessly
        RealRobotAdapter robotDriveAdapter = new RealRobotAdapter();

        //Build all the routes using the adapter so we can select one quickly later
        DirectRoutesExample directRoutesExample = new DirectRoutesExample(robotDriveAdapter);
        directRoutesExample.BuildRoutes();

        while (opModeInInit()) {

            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();

            //TODO what is happening here? Does it work correctly?
            gamepadHandling.lockColorAndSide();
            telemetry.update();
            sleep(10);
        }

        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRoute = directRoutesExample.getRoute(MatchConfig.finalAllianceColor, MatchConfig.finalSideOfField);

        //set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().mecanumDrive.pose= directRoutesExample.getStartingPose(MatchConfig.finalAllianceColor, MatchConfig.finalSideOfField);

        //Reset Gyro
        //TODO i suspect this is not needed or might be duplicative with what is happening in MecanumDrive
        // can we run a test without this and see what happens?
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        //After Init switch the vision processing to AprilTags
        //Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(selectedRoute);

        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees;
        MatchConfig.endOfAutonomousRelativeYawDegrees = Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees;
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getGyroSubsystem().offsetFromAbsoluteYawDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().mecanumDrive.pose;
    }
}

