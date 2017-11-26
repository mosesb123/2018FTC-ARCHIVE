
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: auto Base", group="Pushbot")
public class AutonomousBase extends LinearOpMode {

    /* Declare OpMode boys. */
    HardwareBigBoy robot = new HardwareBigBoy();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    static final double COLOR_ARM_ANGLE = .3; //need to test
    private String teamColor = ""; //our teams color, 2 dif autos


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Big Boy Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        colorActions();
//        cryptoActions(); //unfinished

        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);
        idle();
    }


    private void turnLeft(double x) throws InterruptedException {
        robot.leftFrontMotor.setPower(-(robot.TURN_SPEED + robot.LEFT_MOTOR_OFFSET));
        robot.leftBackMotor.setPower(-(robot.TURN_SPEED + robot.LEFT_MOTOR_OFFSET));
        robot.rightFrontMotor.setPower(robot.TURN_SPEED);
        robot.rightBackMotor.setPower(robot.TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < x)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
        robot.stopMoving();
    }

    private void colorActions() throws InterruptedException { //this all assumes that teamColor == our teams color and the color sensor is in the same direction that forward drive is
        robot.colorServoArm.setPosition(COLOR_ARM_ANGLE);
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        double trueColor = red - blue;
        if (teamColor.compareTo("blue") == 0 && trueColor < 0)
            robot.driveStB();
        else if (teamColor.compareTo("red") == 0 && trueColor > 0)
            robot.driveBtS();
        robot.stopMoving();
    }

//    private void cryptoActions() throws InterruptedException { //finished, none of the other functions are written though
//        int picture = imageSense(); //EASY DOGGY
//        if (imageSense == 1)
//            leftKey(); // drive to put it in the left
//        else if (imageSense == 2)
//            middleKey(); // drive to put it in the middle
//        else
//            rightKey(); //drive to put it in the right
//        robot.stopMoving();
//    }
}