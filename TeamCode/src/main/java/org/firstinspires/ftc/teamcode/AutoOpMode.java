package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
@Disabled
@Autonomous(name="AutoBlueTest", group="Pushbot")
public class AutoOpMode extends LinearOpMode {

    /* Declare OpMode boys. */
    BiggerBoyHardware robot = new BiggerBoyHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private String teamColor = "blue"; //our teams color, 2 dif autos
    OpenGLMatrix lastLocation = null; //Vuforia stuff
    VuforiaLocalizer vuforia; //you'll never guess what this is for


    public void setTeamColor(String color){
        teamColor = color;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AXEsTGf/////AAAAGbF6lsrAgkrrmU3OaMt7gcc7l46IpUxtcXzsdAYiAx7YESYV/QxSwRN72H5y9jgaCjE4lXFjk0K6a6n80oMQhOJ1/siCcfgrEJ1fmI6IHZPm/VAxGi29eLo1ItkuAhpi5apmatTnCamd1be54REtj10OOKPNO2W+ww7UjA23++9Rb55mtU+xRBO2wQd91ugpl6VmkUaQ3cw5YDbqc0v06cmALmoy1x4d6agXpSXDRLm6V1V+r3GYo9g1LdNiB6zSwb+dIwU6e3P8dl9iVGDM3HrBPbf/M/wmEDFEiYEOXa7nQspunnfJKEHckUJU7+qMWqddM9TBpFNLO+ExQK0rAA40plID4wZ9F83qsYh5pCcS";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //front is an option
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Big Boy Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        relicTrackables.activate();


        while (opModeIsActive()) {
            robot.leftServo.setPosition(robot.SLIDE_ARM_HOME);
            robot.rightServo.setPosition(1-robot.SLIDE_ARM_HOME);

            //got a little too object oriented. Vuforia goes here now
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark == RelicRecoveryVuMark.UNKNOWN) { //while loop until we find it

                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "not visible");

                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            telemetry.update();

            colorActions();
            cryptoActions(vuMark.toString());

            robot.leftServo.setPosition(1-robot.SLIDE_ARM_HOME);
            robot.rightServo.setPosition(robot.SLIDE_ARM_HOME);

        }


        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);
        idle();
    }


    private void colorActions() throws InterruptedException { //this all assumes that teamColor == our teams color and the color sensor is in the same direction that forward drive is
        driveRight(.3);
        robot.colorServoArm.setPosition(robot.COLOR_SERVO_DESTINATION);
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        double trueColor = red - blue;
        if (trueColor < 0 /*blue*/) {
            if (teamColor.compareTo("blue") == 0)
                driveStB(.5); //
        }
        else {
            if (teamColor.compareTo("red") == 0)
                driveBtS(.5);
        }
        driveLeft(.3);
        robot.stopMoving();
    }

    private void leftKey() {
        //for all of them, strafe a bit to the left(red) or right(blue) and then go straight. Difference is in how much to strafe
    }
    private void middleKey() {

    }
    private void rightKey() {

    }
    private void cryptoActions(String picture) throws InterruptedException { //finished, none of the other functions are written though
        if (picture == "LEFT")
            leftKey(); // drive to put it in the left
        else if (picture == "CENTER")
            middleKey(); // drive to put it in the middle
        else
            rightKey(); //drive to put it in the right
        robot.stopMoving();
    }

    public void driveStB(double feet) throws InterruptedException {
        driveStraight(feet);
        driveBackwards(feet);
    }
    public void driveBtS(double feet) throws InterruptedException {
        driveBackwards(feet);
        driveStraight(feet);
    }
    public void driveLeft(double feet) {
        double rotations = feet /robot.WHEELS_CIRCUM;
        double ticks = rotations * robot.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition(-1*(int)ticks);
        robot.leftBackMotor.setTargetPosition((int)ticks);
        robot.rightFrontMotor.setTargetPosition((int)ticks);
        robot.rightBackMotor.setTargetPosition(-1*(int)ticks);

        setRunToPosition();

        robot.leftFrontMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.leftBackMotor.setPower(robot.DRIVE_SPEED);
        robot.rightFrontMotor.setPower(robot.DRIVE_SPEED);
        robot.rightBackMotor.setPower(-1*robot.DRIVE_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();
    }
    public void driveRight(double feet){
        double rotations = feet /robot.WHEELS_CIRCUM;
        double ticks = rotations * robot.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition((int)ticks);
        robot.leftBackMotor.setTargetPosition(-1*(int)ticks);
        robot.rightFrontMotor.setTargetPosition(-1*(int)ticks);
        robot.rightBackMotor.setTargetPosition((int)ticks);

        setRunToPosition();

        robot.leftFrontMotor.setPower(robot.DRIVE_SPEED);
        robot.leftBackMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.rightFrontMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.rightBackMotor.setPower(robot.DRIVE_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();
    }
    public void driveStraight(double feet){

        double rotations = feet /robot.WHEELS_CIRCUM;
        double ticks = rotations * robot.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition(-1*(int)ticks);
        robot.leftBackMotor.setTargetPosition(-1*(int)ticks);
        robot.rightFrontMotor.setTargetPosition(-1*(int)ticks);
        robot.rightBackMotor.setTargetPosition(-1*(int)ticks);

        setRunToPosition();

        robot.leftFrontMotor.setPower(robot.DRIVE_SPEED);
        robot.leftBackMotor.setPower(robot.DRIVE_SPEED);
        robot.rightFrontMotor.setPower(robot.DRIVE_SPEED);
        robot.rightBackMotor.setPower(robot.DRIVE_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();

    }
    public void driveBackwards(double feet) {

        double rotations = feet / robot.WHEELS_CIRCUM;
        double ticks = rotations * robot.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition((int) ticks);
        robot.leftBackMotor.setTargetPosition((int) ticks);
        robot.rightFrontMotor.setTargetPosition((int) ticks);
        robot.rightBackMotor.setTargetPosition((int) ticks);

        setRunToPosition();


        robot.leftFrontMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.leftBackMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.rightFrontMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.rightBackMotor.setPower(-1*robot.DRIVE_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();
    }



    public void resetEncoders(){

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setRunToPosition() {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void whileIsBusy() {
        while (robot.leftFrontMotor.isBusy()|| robot.leftBackMotor.isBusy()|| robot.rightFrontMotor.isBusy() || robot.rightBackMotor.isBusy()){}
    }



}
