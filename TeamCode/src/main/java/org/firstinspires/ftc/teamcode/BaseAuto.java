package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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
//@Disabled
@Autonomous(name="BaseAuto", group="Pushbot")
public class BaseAuto extends LinearOpMode {

    /* Declare OpMode boys. */
    private BiggerBoyHardware robot = new BiggerBoyHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private String teamColor = "blue"; //our teams color, 2 dif autos
    private String distance = "close"; //NOTE: close and far is relative to the relic mat
    OpenGLMatrix lastLocation = null; //Vuforia stuff
    VuforiaLocalizer vuforia; //you'll never guess what this is for

    //Constants
    private double slideArmClosedPosition = -0.05; //Closed position to hold onto box
    private double slideArmOpenPosition = 0.8; //Closed position to hold onto box



    public void setTeamColor(String color) { teamColor = color; }
    public void setDistance(String distanceIn) { distance = distanceIn; }
    private boolean vumarkFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Done: Make changes so that code acts differently for "close", "far", "red", "blue"
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AXEsTGf/////AAAAGbF6lsrAgkrrmU3OaMt7gcc7l46IpUxtcXzsdAYiAx7" +
                "YESYV/QxSwRN72H5y9jgaCjE4lXFjk0K6a6n80oMQhOJ1/siCcfgrEJ1fmI6IHZPm/VAxGi29eLo1ItkuA" +
                "hpi5apmatTnCamd1be54REtj10OOKPNO2W+ww7UjA23++9Rb55mtU+xRBO2wQd91ugpl6VmkUaQ3cw5YDb" +
                "qc0v06cmALmoy1x4d6agXpSXDRLm6V1V+r3GYo9g1LdNiB6zSwb+dIwU6e3P8dl9iVGDM3HrBPbf/M/wmED" +
                "FEiYEOXa7nQspunnfJKEHckUJU7+qMWqddM9TBpFNLO+ExQK0rAA40plID4wZ9F83qsYh5pCcS";
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
        telemetry.addData("Status", "Bigger Boy Powered");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.colorServoArm.setPosition(0);
        robot.clawServoArm.setPosition(BiggerBoyHardware.SERVO_MAX);
        relicTrackables.activate();


        while (opModeIsActive()) {



            //got a little too object oriented. Vuforia goes here now
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            runtime.reset();

            while (vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() <= 10) { //while loop until we find it, might take a sec

                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "not visible");

                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
            }

            if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                vumarkFound = true;
            }
            telemetry.addData("VuMark" , "%s", vuMark.toString());
            telemetry.update();

            runtime.reset();
            robot.rightServo.setPosition(slideArmClosedPosition);
            robot.leftServo.setPosition(1-slideArmClosedPosition);
            raiseGlyph(1);
            /*for (int i = 1; i > 0; i-=.1)
                robot.colorServoArm.setPosition(i);
            sleep(1000); //waiting for the servo to get there happy
            int blueReading = robot.colorSensor.blue();
            int redReading = robot.colorSensor.red();
            if (teamColor.equals("blue")) {
                if(blueReading > redReading){
                    driveStB(.3);
                }
                else{
                    driveBtS(.3);
                }
            }
            if (teamColor.equals("red")) {
                if(blueReading < redReading){
                    driveStB(.3);
                }
                else{
                    driveBtS(.3);
                }
            }
            robot.colorServoArm.setPosition(1);
            */
            String key = vuMark.toString(); //TODO Test this, theoretically should work
            if (teamColor.equals("blue")) aboutFace();

            if (distance.equals("close")) {
                if (key.equals("LEFT")) driveStraight(2.8);
                else if (key.equals("RIGHT")) driveStraight(4.8);
                else driveStraight(3.8);
                if (teamColor.equals("blue")) turnLeft();
                if (teamColor.equals("red")) turnRight();
            }
            if (distance.equals("far")) {
                driveStraight(2);
                if (teamColor.equals("blue")) {
                    turnRight();
                    if (key.equals("LEFT")) driveStraight(.5);
                    if (key.equals("RIGHT")) driveStraight(1.5);
                    else driveStraight(1);
                }
                if (teamColor.equals("red")) {
                    turnLeft();
                    if (key.equals("LEFT")) driveStraight(1.5);
                    if (key.equals("RIGHT")) driveStraight(.5);
                    else driveStraight(1);
                }
            }
            driveStraight(2);

            openServos();
            lowerGlyph(1);

            driveBackwards(.5);
            break;
        }


        telemetry.addData("Path", "Complete");

        //telemetry.update();
        sleep(1000);
        idle();
    }

    private void openServos(){
        robot.rightServo.setPosition(slideArmOpenPosition);
        robot.leftServo.setPosition(1-slideArmOpenPosition);
    }

    public void driveStB(double feet) throws InterruptedException {
        driveStraight(feet);
        driveBackwards(feet);
    }
    public void driveBtS(double feet) throws InterruptedException {
        driveBackwards(feet);
        driveStraight(feet);
    }
    public void aboutFace() {
        //TODO implement
        //theoretically just turnLeft twice
        turnLeft();
        turnLeft();
    }
    public void turnLeft() {
        telemetry.addData("Motion", "Turning Right");
        telemetry.update();
        runtime.reset();

        setNoEncoder();

        robot.leftFrontMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.leftBackMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.rightFrontMotor.setPower(-BiggerBoyHardware.TURN_SPEED);
        robot.rightBackMotor.setPower(-BiggerBoyHardware.TURN_SPEED);

        while(runtime.seconds() < 1.5){
            idle();
        }

        robot.stopMoving();

        telemetry.addData("Motion", "Done Turning Right");
        telemetry.update();
    }
    public void turnRight() {
        telemetry.addData("Motion", "Turning Right");
        telemetry.update();
        runtime.reset();

        setNoEncoder();

        robot.leftFrontMotor.setPower(-BiggerBoyHardware.TURN_SPEED);
        robot.leftBackMotor.setPower(-BiggerBoyHardware.TURN_SPEED);
        robot.rightFrontMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.rightBackMotor.setPower(BiggerBoyHardware.TURN_SPEED);

        while(runtime.seconds() < 1.2){
            idle();
        }

        robot.stopMoving();

        telemetry.addData("Motion", "Done Turning Right");
        telemetry.update();
    }
    public void stopMoving() {
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
    }
    /*public void driveLeft(double feet) { //Mech wheels only
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
    public void driveRight(double feet){ // Mech Wheels only
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
    */

    public void raiseGlyph(double feet){

        robot.GlyphMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GlyphMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        runtime.reset();
        while(runtime.seconds() < 1.5)
            idle();

        robot.GlyphMotor.setPower(0);
    }

    public void lowerGlyph(double feet){

        robot.GlyphMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GlyphMotor.setPower(-BiggerBoyHardware.TURN_SPEED);
        runtime.reset();
        while(runtime.seconds() < 1.5)
            idle();

        robot.GlyphMotor.setPower(0);
    }

    public void driveStraight(double feet){
        telemetry.addData("Motion", "Driving Straight");
        telemetry.update();
        double rotations = feet / BiggerBoyHardware.WHEELS_CIRCUM;
        double ticks = rotations * BiggerBoyHardware.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition((int)ticks);
        robot.leftBackMotor.setTargetPosition((int)ticks);
        robot.rightFrontMotor.setTargetPosition((int)ticks);
        robot.rightBackMotor.setTargetPosition((int)ticks);

        setRunToPosition();

        robot.leftFrontMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.leftBackMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.rightFrontMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.rightBackMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();
        telemetry.addData("Motion", "Done Driving Straight");
        telemetry.update();

    }
    public void driveBackwards(double feet) {
        telemetry.addData("Motion", "Driving Backwards");
        telemetry.update();

        double rotations = feet / BiggerBoyHardware.WHEELS_CIRCUM;
        double ticks = rotations * BiggerBoyHardware.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition((int) -ticks);
        robot.leftBackMotor.setTargetPosition((int) -ticks);
        robot.rightFrontMotor.setTargetPosition((int) -ticks);
        robot.rightBackMotor.setTargetPosition((int) -ticks);

        setRunToPosition();


        robot.leftFrontMotor.setPower(-1* BiggerBoyHardware.TURN_SPEED);
        robot.leftBackMotor.setPower(-1* BiggerBoyHardware.TURN_SPEED);
        robot.rightFrontMotor.setPower(-1* BiggerBoyHardware.TURN_SPEED);
        robot.rightBackMotor.setPower(-1* BiggerBoyHardware.TURN_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();

        telemetry.addData("Motion", "Done Driving Backwards");
        telemetry.update();
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

    public void setNoEncoder() {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void whileIsBusy() {
        runtime.reset();
        while ((robot.leftFrontMotor.isBusy()|| robot.leftBackMotor.isBusy()||
                robot.rightFrontMotor.isBusy() || robot.rightBackMotor.isBusy() ) && runtime.seconds() < 5){
            idle();
        }
    }



}
