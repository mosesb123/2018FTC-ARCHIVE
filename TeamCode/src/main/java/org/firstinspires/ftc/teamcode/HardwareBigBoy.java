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
    public DcMotor rightSlideMotor = null;
    public DcMotor leftSlideMotor = null;

    public final static double SLIDE_ARM_HOME = 0.0; //need to test and find, probs 0.0
    public final static double SLIDE_MIN_RANGE = 0.0; //need to test and find, probs 0.0
    public final static double SLIDE_MAX_RANGE = 0.5; //need to test and find, probs 0.5

    public Servo colorServoArm = null;
    public ColorSensor colorSensor = null;
    public final static double DRIVE_SPEED = .8; //TODO find real drive speed
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
        rightSlideMotor = hwmap.dcMotor.get("rightSlideMotor");
        leftSlideMotor= hwmap.dcMotor.get("leftSlideMotor");

        colorServoArm = hwmap.servo.get("colorServoArm");
        colorSensor = hwmap.colorSensor.get("color");


        //Set Mode
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightServoArm.setPosition(SLIDE_ARM_HOME);
        leftServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);

        //Set Power to 0
        stopMoving();

    }


    public void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
        rightServoArm.setPosition(SLIDE_ARM_HOME);
        leftServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);

    }
    public void driveStraight(double x) throws InterruptedException {
        leftFrontMotor.setPower(DRIVE_SPEED );
        leftBackMotor.setPower(DRIVE_SPEED );
        rightFrontMotor.setPower(DRIVE_SPEED);
        rightBackMotor.setPower(DRIVE_SPEED);
        runtime.reset();
        while ((runtime.seconds() < x)) {
            wait();
        }
        stopMoving();
    }
    public void driveBackwards(double x) throws InterruptedException {
        leftFrontMotor.setPower(-1 *DRIVE_SPEED ) ;
        leftBackMotor.setPower(-1 * DRIVE_SPEED  );
        rightFrontMotor.setPower(-1 * DRIVE_SPEED);
        rightBackMotor.setPower(-1 * DRIVE_SPEED);
        runtime.reset();
        while (runtime.seconds() < x) {
            wait();
        }
        stopMoving();
    }
/*
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
    public void driveStB(double time) throws InterruptedException {
        driveStraight(time);
         driveBackwards(time);
    }
    public void driveBtS(double time) throws InterruptedException {
         driveBackwards(time);
         driveStraight(time);
    }

    public void driveStraightDistance(double feet){
        double wheelsCircum = 1.04719755;
        double ticksPerRotation = 1120;
        double rotations = feet / wheelsCircum;
        double ticks = rotations * ticksPerRotation;

        this.leftFrontMotor.setMode(DCMotorController.RunMode.RESET_ENCODERS);
        this.leftBackMotor.setMode(DCMotorController.RunMode.RESET_ENCODERS);
        this.rightFrontMotor.setMode(DCMotorController.RunMode.RESET_ENCODERS);
        this.rightBackMotor.setMode(DCMotorController.RunMode.RESET_ENCODERS);

        this.leftFrontMotor.setTargetPosition(ticks);
        this.leftBackMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);
        this.rightBackMotor.setTargetPosition(ticks);


    }
}


