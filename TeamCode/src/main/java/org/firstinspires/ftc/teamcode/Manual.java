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
@Disabled

public class Manual extends LinearOpMode {

    // Declare OpMode members.
    BiggerBoyHardware robot = new BiggerBoyHardware();
    private ElapsedTime runtime = new ElapsedTime();
    final double DEADZONE = .2;
    double slideArmPosition = 0;
    double armSpeed = 0.5;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap
        );
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive train functions
            if(Math.abs(gamepad1.left_stick_y) >= DEADZONE && Math.abs(gamepad1.left_stick_x)<= DEADZONE) { //Forward Back
                robot.moveSpeedBasic(1, robot.DRIVE_SPEED*gamepad1.left_stick_y);
            }
            else if(Math.abs(gamepad1.left_stick_x) >= DEADZONE && Math.abs(gamepad1.left_stick_y)<= DEADZONE) { //Left Right turn
                robot.moveSpeedBasic(3, robot.DRIVE_SPEED * gamepad1.right_stick_x);
            }
            //NOTE: strafe is not a thing with basic wheels
            else if(Math.abs(gamepad1.left_stick_x) >= DEADZONE && Math.abs(gamepad1.left_stick_y)<= DEADZONE) { //Left Right strafe
                robot.leftFrontMotor.setPower(-robot.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.leftBackMotor.setPower(robot.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.rightBackMotor.setPower(-robot.DRIVE_SPEED * gamepad1.left_stick_x);
                robot.rightFrontMotor.setPower(robot.DRIVE_SPEED * gamepad1.left_stick_x);

            }
            else
                robot.stopMoving();

            //Slide control
            if (gamepad1.left_trigger >= DEADZONE) { //lower slides
                robot.GlyphMotor.setPower(-robot.DRIVE_SPEED); //TODO find correct motor standard power and controller deadzones
            }
            else if (gamepad1.right_trigger >= DEADZONE) //raise slides
            {
                robot.GlyphMotor.setPower(robot.DRIVE_SPEED);
            }
            else
            {
                robot.GlyphMotor.setPower(0);
            }

            //Glyph clamps
            if (gamepad1.left_bumper) {
                slideArmPosition += armSpeed;
                slideArmPosition = Range.clip(slideArmPosition, robot.SERVO_MIN, robot.SERVO_MAX); //make sure position is allowed
                robot.rightServo.setPosition(slideArmPosition); //set position of servos
                robot.leftServo.setPosition(1 - slideArmPosition); //set position of servos
            }
            else if (gamepad1.right_bumper) {
                slideArmPosition -= armSpeed;
                slideArmPosition = Range.clip(slideArmPosition, robot.SERVO_MIN, robot.SERVO_MAX); //make sure position is allowed
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

