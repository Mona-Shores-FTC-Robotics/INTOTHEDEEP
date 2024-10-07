package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Params;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DirectionParams {

    public static void configureChassis19429A(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
        // Set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        //Set motor encoder directions
        driveSubsystem.leftFrontEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        driveSubsystem.leftBackEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        driveSubsystem.rightFrontEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        driveSubsystem.rightBackEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        if (mecanumDrive instanceof PinpointDrive) {
            PinpointDrive.PARAMS = new RRParams.ChassisPinpointParams();
            PinpointDrive.PARAMS.xOffset = 4;
            PinpointDrive.PARAMS.yOffset = 2;
            PinpointDrive.PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_SWINGARM_POD;
            PinpointDrive.PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            PinpointDrive.PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;;
        } else
        {
            MecanumDrive.PARAMS = new RRParams.ChassisPinpointParams();
        }

        if (mecanumDrive.localizer instanceof TwoDeadWheelLocalizer) {
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorEx.Direction.REVERSE);
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }

    // Static method for configuring motor and encoder directions for CenterStage
    public static void configureCenterStageDirections(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
        // Set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Set encoder directions
        driveSubsystem.leftFrontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        driveSubsystem.leftBackEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        driveSubsystem.rightFrontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        driveSubsystem.rightBackEncoder.setDirection(DcMotorEx.Direction.FORWARD);

        // Set deadwheel encoder directions (if applicable)
        if (mecanumDrive.localizer instanceof TwoDeadWheelLocalizer) {
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorEx.Direction.FORWARD);
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorEx.Direction.FORWARD);
        }
    }

    // Add other robot direction configurations as needed
}