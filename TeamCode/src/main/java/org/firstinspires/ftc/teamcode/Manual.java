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


@TeleOp(name="Manual Control", group="Linear Opmode")
//@Disabled

public class Manual extends LinearOpMode {

    // Declare OpMode members.
    BiggerBoyHardware robot = new BiggerBoyHardware();
    private ElapsedTime runtime = new ElapsedTime();
    final double DEADZONE = .2;
    final double SERVO_SPEED = .15;
    double slideArmPosition = 0;
    //double ClawServoXPos = 0;
    double ClawServoYPos = 0;
    double ClawServoZPos = 0;
    double ClawServoArmPos = 0;
    double armSpeed = 0.05;
    double timerstart = 9.87;
    double timer = 9.87; // Should be 10 imo



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.clawServoY.scaleRange(robot.CLAW_SERVO_Y_BUFFER, 1-robot.CLAW_SERVO_Y_BUFFER);

        robot.colorServoArm.setPosition(robot.COLOR_SERVO_HOME);
        robot.leftServo.setPosition(.2);
        robot.rightServo.setPosition(.8);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive train functions
            //todo: multiplying DRIVE_SPEED by left_stick_y might be causing the issue
            if(Math.abs(gamepad1.left_stick_y) >= DEADZONE && Math.abs(gamepad1.left_stick_x)<= DEADZONE) { //Forward Back
//                robot.moveSpeedBasic(MotorDirection.FORWARD, BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_y);
                robot.rightFrontMotor.setPower(-gamepad1.left_stick_y);
                robot.rightBackMotor.setPower(-gamepad1.left_stick_y);
                robot.leftFrontMotor.setPower(-gamepad1.left_stick_y);
                robot.leftBackMotor.setPower(-gamepad1.left_stick_y);
            }
            else if(Math.abs(gamepad1.left_stick_x) >= DEADZONE && Math.abs(gamepad1.left_stick_y)<= DEADZONE) { //Left Right turn
//                robot.moveSpeedBasic(MotorDirection.LEFT, BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.rightFrontMotor.setPower(gamepad1.left_stick_x);
                robot.rightBackMotor.setPower(gamepad1.left_stick_x);
                robot.leftFrontMotor.setPower(-gamepad1.left_stick_x);
                robot.leftBackMotor.setPower(-gamepad1.left_stick_x);
            }

            //Forward
            else if(gamepad1.dpad_up){
                robot.rightFrontMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.rightBackMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftFrontMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftBackMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
            }
            else if(gamepad1.dpad_down){
                robot.rightFrontMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.rightBackMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftFrontMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftBackMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
            }
            else if(gamepad1.dpad_left){
                robot.rightFrontMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.rightBackMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftFrontMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftBackMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
            }
            else if(gamepad1.dpad_right){
                robot.rightFrontMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.rightBackMotor.setPower(.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftFrontMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
                robot.leftBackMotor.setPower(-.5*BiggerBoyHardware.DRIVE_SPEED);
            }


            //NOTE: strafe is not a thing with basic wheels
//            else if(Math.abs(gamepad1.left_stick_x) >= DEADZONE && Math.abs(gamepad1.left_stick_y)<= DEADZONE) { //Left Right strafe
//                robot.leftFrontMotor.setPower(-BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
//                robot.leftBackMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
//                robot.rightBackMotor.setPower(-BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
//                robot.rightFrontMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
//
//            }
            else
                robot.stopMoving();

//            telemetry.addData("timer",timer);
//            telemetry.update();
            //Slide control
            //todo make stopper
//            runtime.startTime();
            if (gamepad1.left_trigger >= DEADZONE /*&& timer < 2 * timerstart*/ ) { //lower slides
//                runtime.reset();
                robot.GlyphMotor.setPower(-BiggerBoyHardware.DRIVE_SPEED);
//                if (runtime.seconds() > 1){
//                    timer++;
//                    runtime.reset();
//                }
//                telemetry.addData("timer",timer);
//                telemetry.update();
            }
            else if (gamepad1.right_trigger >= DEADZONE ) // && timer > 0) //raise slides
            {
//                runtime.reset();

                robot.GlyphMotor.setPower(BiggerBoyHardware.DRIVE_SPEED);

//                if (runtime.seconds() > 1){
//                    timer--;
//                    runtime.reset();
//                }
//                telemetry.addData("timer",timer);
//                telemetry.update();
            }
            else
            {
                robot.GlyphMotor.setPower(0);
                //telemetry.addData("timer",timer);
                //telemetry.update();
            }
//            telemetry.addData("timer",timer);
//            telemetry.update();

            //relic boy motor
            //todo make a stopper so it doesn't break
            while(gamepad1.x) {
                robot.RelicMotor.setPower(-robot.DRIVE_SPEED);
            }
            if (!gamepad1.x) {
                robot.RelicMotor.setPower(0);
            }
            while (gamepad1.y) {
                robot.RelicMotor.setPower(robot.DRIVE_SPEED);
            }
            if (!gamepad1.y) {
                robot.RelicMotor.setPower(0);
            }

            //todo: limit left and right servos so they don't get stuck on side
            //Glyph clamps
            if (gamepad1.left_bumper) {
                slideArmPosition += armSpeed;
                slideArmPosition = Range.clip(slideArmPosition, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.rightServo.setPosition(slideArmPosition); //set position of servos
                robot.leftServo.setPosition(1 - slideArmPosition); //set position of servos
            }
            else if (gamepad1.right_bumper) {
                slideArmPosition -= armSpeed;
                slideArmPosition = Range.clip(slideArmPosition, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.rightServo.setPosition(slideArmPosition); //set position of servos
                robot.leftServo.setPosition(1 - slideArmPosition); //set position of servos
            }
           /* while(Math.abs(gamepad1.right_stick_x) > DEADZONE)
            {
                ClawServoXPos += SERVO_SPEED * gamepad1.right_stick_x;
                ClawServoXPos = Range.clip(ClawServoXPos, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.clawServoX.setPosition(ClawServoXPos);
            } */
            while(Math.abs(gamepad1.right_stick_y) > DEADZONE)
            {
                ClawServoYPos += SERVO_SPEED * gamepad1.right_stick_y;

                ClawServoYPos = Range.clip(ClawServoYPos, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.clawServoY.setPosition(ClawServoYPos);
                //telemetry.addData("ClawServoY", "Attempting to go to %.2f", ClawServoYPos);
                //telemetry.addData("ClawServoY", "Actual Position %.2f", robot.clawServoY.getPosition());
                sleep(50);
            }
            while(Math.abs(gamepad1.right_stick_x) > DEADZONE)
            {
                ClawServoZPos += SERVO_SPEED * gamepad1.right_stick_x;
                ClawServoZPos = Range.clip(ClawServoZPos, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.clawServoZ.setPosition(ClawServoZPos);
            }
            while(gamepad1.a)
            {
                ClawServoArmPos += SERVO_SPEED / 2;
                ClawServoArmPos = Range.clip(ClawServoArmPos, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.clawServoArm.setPosition(ClawServoArmPos);
            }
            while(gamepad1.b)
            {
                ClawServoArmPos -= SERVO_SPEED / 2;
                ClawServoArmPos = Range.clip(ClawServoArmPos, BiggerBoyHardware.SERVO_MIN, BiggerBoyHardware.SERVO_MAX); //make sure position is allowed
                robot.clawServoArm.setPosition(ClawServoArmPos);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("slidearm", "position, %.2f", slideArmPosition);
            // telemetry.addData("SlideMotors", "Both", leftSlidePower); //TODO set powers better
            // telemetry.addData("DriveMotors", "FrontLeft (%.2f), BackLeft (%.2f) ,FrontRight (%.2f,BackRight (%.2f)", leftFrontPower,  leftBackPower,rightFrontPower, rightBackPower);
            telemetry.addData("Let's go Big Boy", "Big Boy");
            telemetry.update();
        }

    }

}

