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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */

    // Drive Motors
    public DcMotor rightFrontMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;





    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        rightFrontMotor  = hwMap.get(DcMotor.class, "rightFrontMotor");
        leftFrontMotor = hwMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor    = hwMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor    = hwMap.get(DcMotor.class, "rightBackMotor");


        // Set all motors to zero power
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        leftFrontMotor.setMode(DcMotot.RunMode.RUN_WITH_ENCODER);
        leftBackMotor.setMode(DcMotot.RunMode.RUN_WITH_ENCODER);
        rightBackMotor.setMode(DcMotot.RunMode.RUN_WITH_ENCODER);
    }

    public void moveSpeedBasic(int direction, int speed){
        telemetry.addData("Status", "About to move");    //
        telemetry.update();
       switch (direction) {
           //Move Foward
           case 1:
               telemtry
               rightFrontMotor.setPower(speed);
               leftFrontMotor.setPower(speed);
               leftBackMotor.setPower(speed);
               rightBackMotor.setPower(speed);
               telemetry.addData("Status", "Moving Foward");    //
               telemetry.update();

               //Move Backward
           case 2:
               rightFrontMotor.setPower(-speed);
               leftFrontMotor.setPower(-speed);
               leftBackMotor.setPower(-speed);
               rightBackMotor.setPower(-speed);
               telemetry.addData("Status", "Moving Backward");    //
               telemetry.update();

               //Turn Left
           case 3:
               rightFrontMotor.setPower(speed);
               leftFrontMotor.setPower(-speed);
               leftBackMotor.setPower(-speed);
               rightBackMotor.setPower(speed);
               telemetry.addData("Status", "Turning Left");    //
               telemetry.update();
               //Turn right
           case 4:
               rightFrontMotor.setPower(-speed);
               leftFrontMotor.setPower(peed);
               leftBackMotor.setPower(speed);
               rightBackMotor.setPower(-speed);
               telemetry.addData("Status", "Turning Right");    //
               telemetry.update();
       }
    }

 }

