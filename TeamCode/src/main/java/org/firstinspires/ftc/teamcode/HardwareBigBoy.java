package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class HardwareBigBoy {
    public DcMotor rightFrontMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;

    public Servo rightServoArm = null;
    public Servo leftServoArm = null;
    public DcMotor rightSlide = null;
    public DcMotor leftSlide = null;

    public final static double SLIDE_ARM_HOME = 0.0; //need to test and find, probs 0.0
    public final static double SLIDE_MIN_RANGE = 0.0; //need to test and find, probs 0.0
    public final static double SLIDE_MAX_RANGE = 0.5; //need to test and find, probs 0.5

    public Servo colorServoArm = null;
    public ColorSensor colorSensor = null;
    public final static double COLOR_ARM_HOME = 0.0; //need to test and find
    public final static double COLOR_ARM_DESTNATION = 0.5; //test it

    public HardwareMap hwmap = null;
    private ElapsedTime runtime = new ElapsedTime(); //idk what it does, just trying to get public methods to work
    

    public HardwareBigBoy() {
    
    }

    public void init(HardwareMap aMap){
        // Hardware Map assignment
        hwmap = aMap;

        //Motor assignments from config in app
        rightFrontMotor = hwmap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hwmap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hwmap.dcMotor.get("leftBackMotor");
        rightBackMotor = hwmap.dcMotor.get("rightBackMotor");

        rightServoArm = hwmap.servo.get("rightServoArm");
        leftServoArm = hwmap.servo.get("leftServoArm");
        rightSlide = hwmap.dcMotor.get("rightSlide");
        leftSlide= hwmap.dcMotor.get("leftSlide");

        colorServoArm = hwmap.servo.get("colorServoArm");
        colorSensor = hwmap.colorSensor.get("color");


        //Set Mode
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightServoArm.setPosition(SLIDE_ARM_HOME);
        leftServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);

        //Set Power to 0
        stopMoving();

    }

/* Braha Testing Zone
    static final double     FORWARD_SPEED = 0.6;
    static final double     LEFT_MOTOR_OFFSET = 0.0; //Probably > 0 because robot moves left when going straight, but maybe we can kish it
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
*/
    public void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightServoArm.setPosition(SLIDE_ARM_HOME);
        leftServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);

    } /*
    public void driveStraight(double x) throws InterruptedException {
        leftFrontMotor.setPower((DRIVE_SPEED + LEFT_MOTOR_OFFSET));
        leftBackMotor.setPower((DRIVE_SPEED + LEFT_MOTOR_OFFSET));
        rightFrontMotor.setPower(DRIVE_SPEED);
        rightBackMotor.setPower(DRIVE_SPEED);
        runtime.reset();
        while ((runtime.seconds() < x)) {
            wait();
        }
        stopMoving();
    }
    public void driveBackwards(double x) throws InterruptedException {
        leftFrontMotor.setPower((-1 * (DRIVE_SPEED + LEFT_MOTOR_OFFSET)));
        leftBackMotor.setPower((-1 * (DRIVE_SPEED + LEFT_MOTOR_OFFSET)));
        rightFrontMotor.setPower(-1 * DRIVE_SPEED);
        rightBackMotor.setPower(-1 * DRIVE_SPEED);
        runtime.reset();
        while (runtime.seconds() < x) {
            wait();
        }
        stopMoving();
    }

    //TODO find the proper time it takes to turn (currently 1)
    public void turnRight() throws InterruptedException {
        leftFrontMotor.setPower(DRIVE_SPEED + LEFT_MOTOR_OFFSET);
        leftBackMotor.setPower(DRIVE_SPEED + LEFT_MOTOR_OFFSET);
        rightFrontMotor.setPower(-1 * DRIVE_SPEED);
        rightBackMotor.setPower(-1 * DRIVE_SPEED);
        runtime.reset();
        while (runtime.seconds() < 1) {
            wait();
        }
        stopMoving();
    }
    public void turnLeft() throws InterruptedException {
        leftFrontMotor.setPower(-1*(DRIVE_SPEED + LEFT_MOTOR_OFFSET));
        leftBackMotor.setPower(-1*(DRIVE_SPEED + LEFT_MOTOR_OFFSET));
        rightFrontMotor.setPower(DRIVE_SPEED);
        rightBackMotor.setPower(DRIVE_SPEED);
        runtime.reset();
        while (runtime.seconds() < 1) {
            wait();
        }
        stopMoving();
    }
    */ //TODO make real driving functions involving mech wheels
    public void driveStB() throws InterruptedException {
        //driveStraight(2.5);
       // driveBackwards(2.5);
    }
    public void driveBtS() throws InterruptedException {
       // driveBackwards(2.5);
       // driveStraight(2.5);
    }
}
