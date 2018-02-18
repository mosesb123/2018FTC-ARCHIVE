package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


enum MotorDirection{
    FORWARD, BACKWARD, LEFT, RIGHT
}

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
    public Servo clawServoY = null;
    public Servo clawServoZ = null;
    public Servo clawServoArm = null;
    public DcMotor RelicMotor = null;
    //Jewel Mechanisms
    public Servo colorServoArm = null;
    public ColorSensor colorSensor = null;

    //Useful Constants //TODO all of these constants need testing + confirmation
    public final static double RIGHT_SERVO_HOME = -1;
    public final static double CLAW_SERVO_Y_BUFFER = .2;
    public final static double RIGHT_SERVO_MIN = -1;
    public final static double RIGHT_SERVO_MAX = .8;
    public final static double LEFT_SERVO_HOME = 1;
    public final static double LEFT_SERVO_MIN = 1;
    public final static double LEFT_SERVO_MAX = -.8;
    public final static double SERVO_MIN = -.8;
    public final static double SERVO_MAX = .8;
    public final static double DRIVE_SPEED = .9;
    public final static double COLOR_SERVO_HOME = 1;
    public final static double COLOR_SERVO_DESTINATION = 0.5;
    public final static double SLIDE_ARM_HOME = 0.0; //need to test and find, probs 0.0
    public final static double ARM_SPEED = .05;
    public final static double WHEELS_CIRCUM = 1.04719755;
    public final static double TICKS_PER_ROTATION = 1120;


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
        //Setting motor directions
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
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
        //TODO: Make sure to initialize the COLOR SENSOR
        // set other motors to RUN_WITHOUT_ENCODERS
        GlyphMotor = ahwMap.dcMotor.get("GlyphMotor");
        RelicMotor = ahwMap.dcMotor.get("RelicMotor");
        rightServo = ahwMap.servo.get("rightServo");
        leftServo = ahwMap.servo.get("leftServo");
        clawServoY = ahwMap.servo.get("clawServoY");
        clawServoZ = ahwMap.servo.get("clawServoZ");
        clawServoArm = ahwMap.servo.get("clawServoArm");
        colorServoArm = ahwMap.servo.get("colorServoArm");
        colorSensor = ahwMap.colorSensor.get("colorSensor");
    }


    /***Other Stuff***/

    public void moveSpeedBasic(MotorDirection direction, double speed) throws InterruptedException { //TODO Make these follow laws of encoders, and put it in a different class
//        telemetry.addData("Status", "About to move");
//        telemetry.update();
       switch (direction) {
           //Move Forward
           case FORWARD:
               rightFrontMotor.setPower(speed);
               rightBackMotor.setPower(speed);
               leftFrontMotor.setPower(speed);
               leftBackMotor.setPower(speed);
//               telemetry.addData("Status", "Moving Forward");
//               telemetry.update();

               //Move Backward
           case BACKWARD:
               rightFrontMotor.setPower(-speed);
               rightBackMotor.setPower(-speed);
               leftFrontMotor.setPower(-speed);
               leftBackMotor.setPower(-speed);
//               telemetry.addData("Status", "Moving Backward");
//               telemetry.update();

               //Turn Left
           case LEFT:
               rightFrontMotor.setPower(speed);
               rightBackMotor.setPower(speed);
               leftFrontMotor.setPower(-speed);
               leftBackMotor.setPower(-speed);
//               telemetry.addData("Status", "Turning Left");
//               telemetry.update();
               //Turn right
           case RIGHT:
               rightFrontMotor.setPower(-speed);
               rightBackMotor.setPower(-speed);
               leftFrontMotor.setPower(speed);
               leftBackMotor.setPower(speed);
//               telemetry.addData("Status", "Turning Right");
//               telemetry.update();
       }
    }
    public void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
//        GlyphMotor.setPower(0);
    }
    public void resetPosition() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        colorServoArm.setPosition(COLOR_SERVO_HOME);
        GlyphMotor.setPower(0);
        rightServo.setPosition(RIGHT_SERVO_HOME);
        leftServo.setPosition(LEFT_SERVO_HOME);
    }
 }

