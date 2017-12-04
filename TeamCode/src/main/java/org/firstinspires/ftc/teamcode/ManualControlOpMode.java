/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
, */

@TeleOp(name="Basic: Big Boy Linear OpMode", group="Linear Opmode")

public class ManualControlOpMode extends LinearOpMode {

    // Declare OpMode members.
    HardwareBigBoy robot = new HardwareBigBoy();
    private ElapsedTime runtime = new ElapsedTime();
    double slideArmPosition = robot.SLIDE_ARM_HOME;
    double colorArmPosition = robot.COLOR_ARM_HOME;
    final double ARM_SPEED = .05;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightFrontMotor  = hardwareMap.get(DcMotor.class, "Right_Front_Motor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_Front_Motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "Left_Back_Motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "Right_Back_Motor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);


        double leftBackPower = 0;
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0 ;
        //motor powers are here idk why


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drivetrain functions
            //moving ifs
            double moveHoriz = gamepad1.left_stick_x;
            double moveVert = gamepad1.left_stick_y;

            if(Math.abs(moveHoriz) <= threshhold;


            leftBackMotor.setPower(leftBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

            if (gamepad1.left_trigger >= .2) //if left Trigger is pressed
                ;//lower slides, *(gamepad1.left_trigger + 1)
             else if (gamepad1.right_trigger >= .2) //if right Trigger is pressed
                ;//raise slides *(gamepad1.right_trigger + 1)
            if(gamepad1.left_bumper) //slide clamps
                slideArmPosition += ARM_SPEED;
            else if(gamepad1.right_bumper)
                slideArmPosition -= ARM_SPEED;

            slideArmPosition = Range.clip(slideArmPosition, robot.SLIDE_MIN_RANGE, robot.SLIDE_MAX_RANGE); //make sure position is allowed
            robot.rightServoArm.setPosition(slideArmPosition); //set position of servos
            robot.leftServoArm.setPosition(slideArmPosition); //set position of servos


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());// value of slide servo arm
            telemetry.addData("Slide motors", "Angle", slideArmPosition);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Let's go Big Boy", "Big Boy");
            telemetry.update();
        }
    }
}
