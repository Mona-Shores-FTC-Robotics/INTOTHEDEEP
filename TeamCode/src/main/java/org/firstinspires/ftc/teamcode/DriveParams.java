package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class DriveParams {
    public static void configureIntoTheDeep19429RRParams() {
        MecanumDrive.PARAMS.logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        MecanumDrive.PARAMS.usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        MecanumDrive.PARAMS.inPerTick = 1;
        MecanumDrive.PARAMS.lateralInPerTick = 0.7145464020405548;
        MecanumDrive.PARAMS.trackWidthTicks = 12.355448896269676;

        // feedforward parameters (in tick units)
        MecanumDrive.PARAMS.kS = 1.1949887108295525;
        MecanumDrive.PARAMS.kV = 0.1256588154090211;
        MecanumDrive.PARAMS.kA = 0.03;

        // path profile parameters (in inches)
        MecanumDrive.PARAMS.maxWheelVel = 32;
        MecanumDrive.PARAMS.minProfileAccel = -32;
        MecanumDrive.PARAMS.maxProfileAccel = 32;

        // turn profile parameters (in radians)
        MecanumDrive.PARAMS.maxAngVel = Math.PI; // shared with path
        MecanumDrive.PARAMS.maxAngAccel = Math.PI;

        // path controller gains
        MecanumDrive.PARAMS.axialGain = 8;
        MecanumDrive.PARAMS.lateralGain = 1;
        MecanumDrive.PARAMS.headingGain = 6; // shared with turn

        MecanumDrive.PARAMS.axialVelGain = .1;
        MecanumDrive.PARAMS.lateralVelGain = 0;
        MecanumDrive.PARAMS.headingVelGain = 0; // shared with turn

        PinpointDrive.PARAMS.xOffset = -5;
        PinpointDrive.PARAMS.yOffset = 3;
        PinpointDrive.PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;
        PinpointDrive.PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointDrive.PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    }

    public static void configureIntoTheDeep19429Directions(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
        // Set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        //Set motor encoder directions
        if (mecanumDrive.localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer localizer = (MecanumDrive.DriveLocalizer) mecanumDrive.localizer;
            localizer.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            localizer.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            localizer.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            localizer.rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        }
    }

    public static void configureIntoTheDeep20245RRParams() {
        // Assuming you want to configure a separate set of parameters for another robot
        // If MecanumDrive.PARAMS is shared, ensure that configurations don't conflict
        // Alternatively, consider creating a separate Params object for each configuration

        MecanumDrive.PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        MecanumDrive.PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Drive model parameters
        MecanumDrive.PARAMS.inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
        MecanumDrive.PARAMS.lateralInPerTick = 0.5986580181755985;
        MecanumDrive.PARAMS.trackWidthTicks = 12.734772672832092;

        // Feedforward parameters (in tick units)
        MecanumDrive.PARAMS.kS = 1.5603878434857292;
        MecanumDrive.PARAMS.kV = 0.12440377818594824;
        MecanumDrive.PARAMS.kA = 0.03;

        // Path profile parameters (in inches)
        MecanumDrive.PARAMS.maxWheelVel = 45;
        MecanumDrive.PARAMS.minProfileAccel = -45;
        MecanumDrive.PARAMS.maxProfileAccel = 45;

        // Turn profile parameters (in radians)
        MecanumDrive.PARAMS.maxAngVel = Math.toRadians(270); // Shared with path
        MecanumDrive.PARAMS.maxAngAccel = Math.toRadians(270);;

        // Path controller gains
        MecanumDrive.PARAMS.axialGain = 8;
        MecanumDrive.PARAMS.lateralGain = 8;
        MecanumDrive.PARAMS.headingGain = 6; // Shared with turn

        // Velocity controller gains
        MecanumDrive.PARAMS.axialVelGain = 0.1;
        MecanumDrive.PARAMS.lateralVelGain = 0.0;
        MecanumDrive.PARAMS.headingVelGain = 0.0; // Shared with turn
    }


    public static void configureIntoTheDeep20245Directions(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
        // Set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        //Set motor encoder directions
        if (mecanumDrive.localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer localizer = (MecanumDrive.DriveLocalizer) mecanumDrive.localizer;
            localizer.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            localizer.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            localizer.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            localizer.rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        }
    }

}

