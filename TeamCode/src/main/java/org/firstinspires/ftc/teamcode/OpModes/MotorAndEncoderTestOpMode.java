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

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@TeleOp(name = "Motor and Encoder Test", group = "Test")
public class MotorAndEncoderTestOpMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the robot and hardware
        Robot.createInstance(this, Robot.RobotType.ROBOT_CHASSIS_TWO_DEAD_WHEEL_INTERNAL_IMU);
        Robot.getInstance().init(Robot.OpModeType.TELEOP);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        MecanumDrive mecanumDrive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();

        TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) mecanumDrive.localizer;
        Encoder parEncoder = dl.par;
        Encoder perpEncoder = dl.perp;


        while (opModeIsActive()) {
            // Set motor power based on gamepad input (simple test for directions)
            double power = gamepad1.left_stick_y;
            mecanumDrive.leftFront.setPower(power);
            mecanumDrive.leftBack.setPower(power);
            mecanumDrive.rightFront.setPower(power);
            mecanumDrive.rightBack.setPower(power);

            // Log motor power and encoder positions
            telemetry.addData("Motor Power", "LF (%.2f), LB (%.2f), RF (%.2f), RB (%.2f)",
                    mecanumDrive.leftFront.getPower(), mecanumDrive.leftBack.getPower(), mecanumDrive.rightFront.getPower(), mecanumDrive.rightBack.getPower());

            telemetry.addData("Motor Encoder Positions", "LF (%d), LB (%d), RF (%d), RB (%d)",
                    mecanumDrive.leftFront.getCurrentPosition(), mecanumDrive.leftBack.getCurrentPosition(),
                    mecanumDrive.rightFront.getCurrentPosition(), mecanumDrive.rightBack.getCurrentPosition());

            telemetry.addData("Motor Encoder Velocities", "LF (%d), LB (%d), RF (%d), RB (%d)",
                    mecanumDrive.leftFront.getVelocity(), mecanumDrive.leftBack.getVelocity(),
                    mecanumDrive.rightFront.getVelocity(), mecanumDrive.rightBack.getVelocity());

            // Log dead wheel encoder positions
            telemetry.addData("Dead Wheel Encoder Positions", "Parallel (%d), Perpendicular (%d)",
                    parEncoder.getPositionAndVelocity().position, perpEncoder.getPositionAndVelocity().position);

            // Log dead wheel encoder positions
            telemetry.addData("Dead Wheel Encoder Velocities", "Parallel (%d), Perpendicular (%d)",
                    parEncoder.getPositionAndVelocity().velocity, perpEncoder.getPositionAndVelocity().velocity);

            // Log dead wheel encoder positions
            telemetry.addData("Dead Wheel Encoder Directions", "Parallel (%d), Perpendicular (%d)",
                    parEncoder.getDirection(), perpEncoder.getDirection());

            telemetry.update();
        }
    }

}
