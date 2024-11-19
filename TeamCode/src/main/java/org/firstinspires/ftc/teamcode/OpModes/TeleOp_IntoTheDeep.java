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

import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.hasAutoRun;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.IntoTheDeepDriverBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.IntoTheDeepOperatorBindings;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@TeleOp(name="TeleOp_IntoTheDeep")
public class TeleOp_IntoTheDeep extends LinearOpMode
{
    private IntoTheDeepDriverBindings driverBindings;

    @Override
    public void runOpMode()
    {
        //Reset the Singleton CommandScheduler
        CommandScheduler.getInstance().reset();

        // Create the robot
        Robot.createInstance(this);

        // Initialize the robot
        Robot.getInstance().init(Robot.OpModeType.TELEOP);

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        telemetry.clearAll();

        while (opModeInInit()) {
            gamepadHandling.getDriverGamepad().readButtons();

            // Allow driver to override and lock alliance color and side
            gamepadHandling.SelectAllianceAndSide(telemetry);

            telemetry.update();
            sleep(10);
        }

            //todo can we make this contingent on an auto being run or not? Anything else we should think about based on if an auto was run?
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE) && !hasAutoRun) {
            //set the starting location of the robot on the field
            Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);
            //After we set the start pose, use that to set the offset from the start pose for field centric driving
//            Robot.getInstance().getDriveSubsystem().CalculateYawOffset();
            Robot.getInstance().getDriveSubsystem().CalculateYawOffset();
        }

        // Setup Button Bindings
        // Example for avoiding duplicate bindings
        driverBindings = new IntoTheDeepDriverBindings(gamepadHandling.getDriverGamepad(), gamepadHandling.getBindingManager());
        new IntoTheDeepOperatorBindings(gamepadHandling.getOperatorGamepad(), gamepadHandling.getBindingManager());

        //Start the TeleOp Timer
        MatchConfig.teleOpTimer = new ElapsedTime();
        MatchConfig.teleOpTimer.reset();

        MatchConfig.loopTimer = new ElapsedTime();
        MatchConfig.loopTimer.reset();

        MatchConfig.telemetryPacket = new TelemetryPacket();

        //Turn the lights off to start
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.LIGHTING))
        {
            Robot.getInstance().getLightingSubsystem().setLightBlack();
        }

        while (opModeIsActive())
        {
            // Add the loop time to the sliding window average
            MatchConfig.addLoopTime( MatchConfig.loopTimer.milliseconds());

            // Reset the loop timer
            MatchConfig.loopTimer.reset();

            // Run the scheduler
            CommandScheduler.getInstance().run();

            // Read buttons
            gamepadHandling.getDriverGamepad().readButtons();
//            driverBindings.updateTriggerBindings();

            // Display Telemetry through the Robot's Telemetry Manager
            Robot.getInstance().getDriverStationTelemetryManager().displayTelemetry(gamepadHandling.getBindingManager());

            // Send packet to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(MatchConfig.telemetryPacket);

            // Clear the packet for the next loop
            MatchConfig.telemetryPacket = new TelemetryPacket();
        }
    }
}
