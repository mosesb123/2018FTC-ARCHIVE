package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="OLD-AutoBC", group="Pushbot")
public class OLD-AutoBC extends LinearOpMode {

    /* Declare OpMode boys. */
    HardwareBigBoy robot = new HardwareBigBoy();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private String teamColor = "blue"; //our teams color, 2 dif autos
    OpenGLMatrix lastLocation = null; //Vuforia stuff
    VuforiaLocalizer vuforia; //you'll never guess what this is for
    public final static double SLIDE_ARM_HOME = 0.0; //need to test and find, probs 0.0
    public final static double DRIVE_SPEED = .9; //TODO find real drive speed
    public final static double COLOR_ARM_HOME = 0; //need to test and find
    public final static double COLOR_ARM_DESTNATION = 0; //test it
    final double ARM_SPEED = .05;
    final double MOTOR_SPEED = .8;
    final double WHEELS_CIRCUM = 1.04719755;
    final double TICKS_PER_ROTATION = 1120;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftSlideMotor = null;
    private DcMotor rightSlideMotor = null;
    private Servo leftServoArm = null;
    private Servo rightServoArm = null;
    private Servo colorServoArm = null;
    public ColorSensor colorSensor = null;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AXEsTGf/////AAAAGbF6lsrAgkrrmU3OaMt7gcc7l46IpUxtcXzsdAYiAx7YESYV/QxSwRN72H5y9jgaCjE4lXFjk0K6a6n80oMQhOJ1/siCcfgrEJ1fmI6IHZPm/VAxGi29eLo1ItkuAhpi5apmatTnCamd1be54REtj10OOKPNO2W+ww7UjA23++9Rb55mtU+xRBO2wQd91ugpl6VmkUaQ3cw5YDbqc0v06cmALmoy1x4d6agXpSXDRLm6V1V+r3GYo9g1LdNiB6zSwb+dIwU6e3P8dl9iVGDM3HrBPbf/M/wmEDFEiYEOXa7nQspunnfJKEHckUJU7+qMWqddM9TBpFNLO+ExQK0rAA40plID4wZ9F83qsYh5pCcS";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //front is an option
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Big Boy Ready to run");    //
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

        rightServoArm = hardwareMap.servo.get("rightServoArm");
        leftServoArm = hardwareMap.servo.get("leftServoArm");
        colorServoArm = hardwareMap.servo.get("colorServoArm");
        colorSensor = hardwareMap.colorSensor.get("color");


        double leftBackPower = 0;
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0 ;
        double leftSlidePower = 0;
        double rightSlidePower = 0;
        leftServoArm.setPosition(1-SLIDE_ARM_HOME);
        rightServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        relicTrackables.activate();


        while (opModeIsActive()) {
            leftServoArm.setPosition(SLIDE_ARM_HOME);
            rightServoArm.setPosition(1-SLIDE_ARM_HOME);

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

            leftServoArm.setPosition(1-SLIDE_ARM_HOME);
            rightServoArm.setPosition(SLIDE_ARM_HOME);

        }


        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);
        idle();
    }


    private void colorActions() throws InterruptedException { //this all assumes that teamColor == our teams color and the color sensor is in the same direction that forward drive is
        driveRight(.3);
        colorServoArm.setPosition(COLOR_ARM_DESTNATION);
        double red = colorSensor.red();
        double blue = colorSensor.blue();
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
        stopMoving();
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
        stopMoving();
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
        double rotations = feet /WHEELS_CIRCUM;
        double ticks = rotations * TICKS_PER_ROTATION;
        resetEncoders();

        leftFrontMotor.setTargetPosition(-1*(int)ticks);
        leftBackMotor.setTargetPosition((int)ticks);
        rightFrontMotor.setTargetPosition((int)ticks);
        rightBackMotor.setTargetPosition(-1*(int)ticks);

        setRunToPosition();

        leftFrontMotor.setPower(-1*MOTOR_SPEED);
        leftBackMotor.setPower(MOTOR_SPEED);
        rightFrontMotor.setPower(MOTOR_SPEED);
        rightBackMotor.setPower(-1*MOTOR_SPEED);
        whileIsBusy();
        stopMoving();
        resetEncoders();
    }
    public void driveRight(double feet){
        double rotations = feet /WHEELS_CIRCUM;
        double ticks = rotations * TICKS_PER_ROTATION;
        resetEncoders();

        leftFrontMotor.setTargetPosition((int)ticks);
        leftBackMotor.setTargetPosition(-1*(int)ticks);
        rightFrontMotor.setTargetPosition(-1*(int)ticks);
        rightBackMotor.setTargetPosition((int)ticks);

        setRunToPosition();

        leftFrontMotor.setPower(MOTOR_SPEED);
        leftBackMotor.setPower(-1*MOTOR_SPEED);
        rightFrontMotor.setPower(-1*MOTOR_SPEED);
        rightBackMotor.setPower(MOTOR_SPEED);
        whileIsBusy();
        stopMoving();
        resetEncoders();
    }
    public void driveStraight(double feet){

        double rotations = feet /WHEELS_CIRCUM;
        double ticks = rotations * TICKS_PER_ROTATION;
        resetEncoders();

        leftFrontMotor.setTargetPosition(-1*(int)ticks);
        leftBackMotor.setTargetPosition(-1*(int)ticks);
        rightFrontMotor.setTargetPosition(-1*(int)ticks);
        rightBackMotor.setTargetPosition(-1*(int)ticks);

        setRunToPosition();

        leftFrontMotor.setPower(MOTOR_SPEED);
        leftBackMotor.setPower(MOTOR_SPEED);
        rightFrontMotor.setPower(MOTOR_SPEED);
        rightBackMotor.setPower(MOTOR_SPEED);
        whileIsBusy();
        stopMoving();
        resetEncoders();

    }
    public void driveBackwards(double feet) {

        double rotations = feet / WHEELS_CIRCUM;
        double ticks = rotations * TICKS_PER_ROTATION;
        resetEncoders();

        leftFrontMotor.setTargetPosition((int) ticks);
        leftBackMotor.setTargetPosition((int) ticks);
        rightFrontMotor.setTargetPosition((int) ticks);
        rightBackMotor.setTargetPosition((int) ticks);

        setRunToPosition();


        leftFrontMotor.setPower(-1*MOTOR_SPEED);
        leftBackMotor.setPower(-1*MOTOR_SPEED);
        rightFrontMotor.setPower(-1*MOTOR_SPEED);
        rightBackMotor.setPower(-1*MOTOR_SPEED);
        whileIsBusy();
        stopMoving();
        resetEncoders();
    }

    public void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
    }

    public void resetEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setRunToPosition() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void whileIsBusy() {
        while (leftFrontMotor.isBusy()|| leftBackMotor.isBusy()|| rightFrontMotor.isBusy() || rightBackMotor.isBusy()){}
    }



}
