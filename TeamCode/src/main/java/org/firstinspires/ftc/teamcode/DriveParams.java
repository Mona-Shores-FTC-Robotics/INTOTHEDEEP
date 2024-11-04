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
        MecanumDrive.PARAMS.kA = 0.03 ;

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
}

