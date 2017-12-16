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


@TeleOp(name="BigBoy OpMode", group="Linear Opmode")


public class ManualControlOpMode extends LinearOpMode {

    // Declare OpMode members.
    HardwareBigBoy robot = new HardwareBigBoy();
    private ElapsedTime runtime = new ElapsedTime();
    public final static double DRIVE_SPEED = .9; //TODO find real drive speed
    public final static double COLOR_ARM_HOME = 0; //need to test and find
    double slideArmPosition = 0;
    final double ARM_SPEED = .05;
    final double MOTOR_SPEED = .8;
    final double DEADZONE = .2;
    public final static double SLIDE_MIN_RANGE = 0; //need to test and find
    public final static double SLIDE_MAX_RANGE = 1; //need to test and find
    private DcMotor rightFrontMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftSlideMotor = null;
    private DcMotor rightSlideMotor = null;
    private Servo   leftServoArm = null;
    private Servo   rightServoArm = null;
    private Servo   colorServoArm = null;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightFrontMotor  = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftSlideMotor= hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightServoArm = hardwareMap.get(Servo.class, "rightServoArm");
        leftServoArm = hardwareMap.get(Servo.class, "leftServoArm");
        colorServoArm = hardwareMap.servo.get("colorServoArm");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        colorServoArm.setPosition(1);
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive train functions
            if(Math.abs(gamepad1.left_stick_y) >= DEADZONE && Math.abs(gamepad1.left_stick_x)<= DEADZONE) { //Forward Back
                leftFrontMotor.setPower(MOTOR_SPEED * gamepad1.left_stick_y);
                rightFrontMotor.setPower(MOTOR_SPEED * gamepad1.left_stick_y);
                leftBackMotor.setPower(MOTOR_SPEED * gamepad1.left_stick_y);
                rightBackMotor.setPower(MOTOR_SPEED * gamepad1.left_stick_y);
            }
            else if(Math.abs(gamepad1.left_stick_x) >= DEADZONE && Math.abs(gamepad1.left_stick_y)<= DEADZONE) { //Left Right strafe
                leftFrontMotor.setPower(-MOTOR_SPEED * gamepad1.left_stick_x);
                leftBackMotor.setPower(MOTOR_SPEED * gamepad1.left_stick_x);
                rightBackMotor.setPower(-MOTOR_SPEED * gamepad1.left_stick_x);
                rightFrontMotor.setPower(MOTOR_SPEED * gamepad1.left_stick_x);

            }

            else if(Math.abs(gamepad1.right_stick_x) >= DEADZONE && Math.abs(gamepad1.right_stick_y)<= DEADZONE) { //Left Right turn
                leftFrontMotor.setPower(-MOTOR_SPEED * gamepad1.right_stick_x);
                leftBackMotor.setPower(-MOTOR_SPEED * gamepad1.right_stick_x);
                rightBackMotor.setPower(MOTOR_SPEED * gamepad1.right_stick_x);
                rightFrontMotor.setPower(MOTOR_SPEED * gamepad1.right_stick_x);

            }
            else
                stopMoving();
            //Slide control
            if (gamepad1.left_trigger >= DEADZONE) { //if left Trigger is pressed lower slides
                leftSlideMotor.setPower(-MOTOR_SPEED); //TODO find correct motor standard power and controller deadzones
                rightSlideMotor.setPower(-MOTOR_SPEED);
            }
            else if (gamepad1.right_trigger >= DEADZONE) //if right Trigger is pressed raise slides
            {
                leftSlideMotor.setPower(MOTOR_SPEED);
                rightSlideMotor.setPower(MOTOR_SPEED);
            }
            else
            {
                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);
            }


            if (gamepad1.left_bumper) {//slide clamps
                slideArmPosition += ARM_SPEED;
                slideArmPosition = Range.clip(slideArmPosition, SLIDE_MIN_RANGE, SLIDE_MAX_RANGE); //make sure position is allowed
                rightServoArm.setPosition(slideArmPosition); //set position of servos
                leftServoArm.setPosition(1 - slideArmPosition); //set position of servos
            }
            else if (gamepad1.right_bumper) {
                slideArmPosition -= ARM_SPEED;
                slideArmPosition = Range.clip(slideArmPosition, SLIDE_MIN_RANGE, SLIDE_MAX_RANGE); //make sure position is allowed
                rightServoArm.setPosition(slideArmPosition); //set position of servos
                leftServoArm.setPosition(1 - slideArmPosition); //set position of servos
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SlideServos", "Angle", slideArmPosition);
           // telemetry.addData("SlideMotors", "Both", leftSlidePower); //TODO set powers better
           // telemetry.addData("DriveMotors", "FrontLeft (%.2f), BackLeft (%.2f) ,FrontRight (%.2f,BackRight (%.2f)", leftFrontPower,  leftBackPower,rightFrontPower, rightBackPower);
            telemetry.addData("Let's go Big Boy", "Big Boy");
            telemetry.update();
        }

    }
    private void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
    }
}
/*
            //moving ifs
            double moveHoriz = gamepad1.left_stick_x;
            double moveVert = gamepad1.left_stick_y;
            double turnHoriz = gamepad1.right_stick_x;
            double turnVert = gamepad1.right_stick_y;

            if(Math.abs(moveHoriz) <= DEADZONE && Math.abs(moveVert) >= DEADZONE) //move forwards or backwards
            {
                leftFrontPower=(moveVert);
                rightFrontPower=(moveVert);
                leftBackPower=(moveVert);
                rightBackPower=(moveVert);
            }
            else if(Math.abs(moveHoriz) >= DEADZONE && Math.abs(moveVert) <= DEADZONE) //move right or left
            {
                leftFrontPower=(moveHoriz);
                rightBackPower=(moveHoriz);
            }
            else if(moveHoriz >= DEADZONE && moveVert >= DEADZONE) //move right up or left down
            {
                leftFrontPower=(moveVert*moveHoriz*(moveHoriz/Math.abs(moveHoriz))); // Math.abs item so that we know if we are reversing or forward
                rightBackPower=(moveVert*moveHoriz*(moveHoriz/Math.abs(moveHoriz)));
            }
            else if(moveHoriz <= -1 * DEADZONE && moveVert <= -1 * DEADZONE) //move right down or left up
            {
                leftFrontPower=(-1 * moveVert*moveHoriz*(moveVert/Math.abs(moveVert)));
                rightBackPower=(-1 * moveVert*moveHoriz*(moveVert/Math.abs(moveVert)));
            }
            //turning ifs
            else if (gamepad1.right_stick_button) //turn around
            {
                leftFrontPower= -1 * MOTOR_SPEED;
                rightFrontPower=MOTOR_SPEED;
                leftBackPower=-1 *MOTOR_SPEED;
                rightBackPower=MOTOR_SPEED;
            }
            else if(turnHoriz >= DEADZONE && moveVert >= DEADZONE) // turn right and left off of different wheels
                rightFrontPower= (turnHoriz*turnVert);
            else if(turnHoriz <= -1 * DEADZONE && moveVert >= DEADZONE)
                leftFrontPower=(Math.abs(turnHoriz*turnVert));
            else if(turnHoriz >= DEADZONE && moveVert <= -1 * DEADZONE)
                rightBackPower=(Math.abs(turnHoriz*turnVert));
            else if(turnHoriz <= -1 * DEADZONE && moveVert <= -1 * DEADZONE)
                leftBackPower=(turnHoriz*turnVert);

            else if(Math.abs(turnHoriz) <= DEADZONE && Math.abs(turnVert) >= DEADZONE) //turn on front and rear axis
            {
                if (turnVert >= 1)
                {
                    leftBackPower = -1* MOTOR_SPEED;
                    rightBackPower = MOTOR_SPEED;
                }
                else
                    leftFrontPower = MOTOR_SPEED;
                rightFrontPower = -1 * MOTOR_SPEED;
            }
            else {
                leftBackPower = 0;
                leftFrontPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
            }
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftFrontMotor.setPower(leftFrontPower);

            //Slide control
            if (gamepad1.left_trigger >= DEADZONE) { //if left Trigger is pressed lower slides
                leftSlidePower = -1 * MOTOR_SPEED; //TODO find correct motor standard power and controller deadzones
                rightSlidePower = MOTOR_SPEED;
            }
            else if (gamepad1.right_trigger >= DEADZONE) //if right Trigger is pressed raise slides
            {
                leftSlidePower = MOTOR_SPEED;
                rightSlidePower = -1 * MOTOR_SPEED;
            }
            else
            {
                leftSlidePower = 0;
                rightSlidePower = 0;
            }
            rightSlideMotor.setPower(rightSlidePower);
            leftSlideMotor.setPower(leftSlidePower);
 */