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
    double slideArmPosition = 0;
    double armSpeed = 0.5;
    double timerstart = 9.87;
    double timer = 9.87;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive train functions
            //todo: multiplying DRIVE_SPEED by left_stick_y might be causing the issue
            if(Math.abs(gamepad1.left_stick_y) >= DEADZONE && Math.abs(gamepad1.left_stick_x)<= DEADZONE) { //Forward Back
//                robot.moveSpeedBasic(MotorDirection.FORWARD, BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_y);
                robot.rightFrontMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_y);
                robot.rightBackMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_y);
                robot.leftFrontMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_y);
                robot.leftBackMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_y);
            }
            else if(Math.abs(gamepad1.left_stick_x) >= DEADZONE && Math.abs(gamepad1.left_stick_y)<= DEADZONE) { //Left Right turn
//                robot.moveSpeedBasic(MotorDirection.LEFT, BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.rightFrontMotor.setPower(-BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.rightBackMotor.setPower(-BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.leftFrontMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.leftBackMotor.setPower(BiggerBoyHardware.DRIVE_SPEED * gamepad1.left_stick_x);
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
            if (gamepad1.left_trigger >= DEADZONE && timer < 2 * timerstart) { //lower slides
//                runtime.reset();
                robot.GlyphMotor.setPower(-BiggerBoyHardware.DRIVE_SPEED);
//                if (runtime.seconds() > 1){
//                    timer++;
//                    runtime.reset();
//                }
//                telemetry.addData("timer",timer);
//                telemetry.update();
            }
            else if (gamepad1.right_trigger >= DEADZONE && timer > 0) //raise slides
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
                telemetry.addData("timer",timer);
                telemetry.update();
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



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SlideServos", "Angle", slideArmPosition);
            // telemetry.addData("SlideMotors", "Both", leftSlidePower); //TODO set powers better
            // telemetry.addData("DriveMotors", "FrontLeft (%.2f), BackLeft (%.2f) ,FrontRight (%.2f,BackRight (%.2f)", leftFrontPower,  leftBackPower,rightFrontPower, rightBackPower);
            telemetry.addData("Let's go Big Boy", "Big Boy");
            telemetry.update();
        }

    }

}

