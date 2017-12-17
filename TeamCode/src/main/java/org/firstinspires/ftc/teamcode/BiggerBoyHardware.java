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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;



public class BiggerBoyHardware
{
    // Drive Motors
    public DcMotor rightFrontMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    //Glyph Mechanisms
    public Servo rightServo = null;
    public Servo leftServo = null;
    public DcMotor GlyphMotor = null;
    //Relic Mechanisms
    public Servo clawServo = null;
    public DcMotor RelicMotor = null;
    //Jewel Mechanisms
    public Servo colorServo = null;
    public ColorSensor colorSensor = null;
    //Useful Constants //TODO all of these constants need testing + conformation
    public final static double RIGHT_SERVO_HOME = 1;
    public final static double RIGHT_SERVO_MIN = -1;
    public final static double RIGHT_SERVO_MAX = 1;
    public final static double LEFT_SERVO_HOME = -1;
    public final static double LEFT_SERVO_MIN = 1;
    public final static double LEFT_SERVO_MAX = -1;
    public final static double DRIVE_SPEED = .9;
    public final static double COLOR_SERVO_HOME = 0;
    public final static double COLOR_SERVO_DESTNATION = 0.5;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public BiggerBoyHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize DRIVE Motors
        rightFrontMotor = ahwMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = ahwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = ahwMap.dcMotor.get("leftBackMotor");
        rightBackMotor = ahwMap.dcMotor.get("rightBackMotor");
        // Set drive motors to zero power
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        // Set drive motors to run USING encoders.
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO Define and Initialize GLYPH,RELIC,COlOR items: same syntax as above, only motors need setpower 0. not doing it now bc we don't know how many motors we are using for each thing
        // set other motors to RUN_WITHOUT_ENCODERS
    }

    public void moveSpeedBasic(int direction, int speed){ //TODO Make these follow laws of encoders, and put it in a different class
        telemetry.addData("Status", "About to move");    //
        telemetry.update();
       switch (direction) {
           //Move Forward
           case 1:
               rightFrontMotor.setPower(speed);
               leftFrontMotor.setPower(speed);
               leftBackMotor.setPower(speed);
               rightBackMotor.setPower(speed);
               telemetry.addData("Status", "Moving Forward");    //
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
               leftFrontMotor.setPower(speed);
               leftBackMotor.setPower(speed);
               rightBackMotor.setPower(-speed);
               telemetry.addData("Status", "Turning Right");    //
               telemetry.update();
       }
    }

 }

