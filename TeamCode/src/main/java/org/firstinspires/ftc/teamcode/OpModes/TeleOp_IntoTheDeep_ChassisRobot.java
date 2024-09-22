/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.IntoTheDeepDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings.IntoTheDeepOperatorBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name="TeleOp_IntoTheDeep")
public class TeleOp_IntoTheDeep_ChassisRobot extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        //Reset the Singleton CommandScheduler
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        // Create the robot
        Robot.createInstance(this, Robot.RobotType.ROBOT_CHASSIS);

        // Initialize the robot
        Robot.getInstance().init(Robot.OpModeType.TELEOP);

        // Setup Button Bindings
        new IntoTheDeepDriverBindings(gamepadHandling.getDriverGamepad());
        new IntoTheDeepOperatorBindings(gamepadHandling.getOperatorGamepad());

        telemetry.clearAll();

        while (opModeInInit()) {

//            VisionTelemetry.telemetryForInitProcessing(gamepadHandling);
            gamepadHandling.getDriverGamepad().readButtons();
//            gamepadHandling.lockColorAndSide();

            telemetry.update();
            sleep(10);
        }

        //Switch the vision processing to AprilTags
//        Robot.getInstance().getVisionSubsystem().SwitchToAprilTagProcessor();

        //Reset Gyro and pose to be 0 at whatever heading the robot is at
        //TODO experiment with what the robot does without this
        Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        //Set the flag so we reset the gyro/pose heading to zero the next time we go to the backdrop
//        Robot.getInstance().getVisionSubsystem().resetHeading=true;

        //Start the TeleOp Timer
        MatchConfig.teleOpTimer = new ElapsedTime();
        MatchConfig.teleOpTimer.reset();

        MatchConfig.loopTimer = new ElapsedTime();
        MatchConfig.loopTimer.reset();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        MatchConfig.telemetryPacket = new TelemetryPacket();
        while (opModeIsActive())
        {
            Robot.getInstance().getDriveSubsystem().getMecanumDrive().LoopDriverStationTelemetry(telemetry);

            //Reset the timer for the loop timer
            MatchConfig.loopTimer.reset();

            //Run the Scheduler
            CommandScheduler.getInstance().run();

            //Read all buttons
            gamepadHandling.getDriverGamepad().readButtons();

            //Activate End Game Rumble at 87 seconds into TeleOp
            gamepadHandling.endGameRumble();

            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(MatchConfig.telemetryPacket);
            MatchConfig.telemetryPacket = new TelemetryPacket();
        }
    }
}