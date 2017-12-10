package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public final static double SLIDE_ARM_HOME = 1; //need to test and find, probs 0.0
    public final static double SLIDE_MIN_RANGE = -1; //need to test and find, probs 0.0
    public final static double SLIDE_MAX_RANGE = 1; //need to test and find, probs 0.5

    public Servo colorServoArm = null;
    public ColorSensor colorSensor = null;
    public final static double DRIVE_SPEED = .9; //TODO find real drive speed
    public final static double COLOR_ARM_HOME = 0; //need to test and find
    public final static double COLOR_ARM_DESTNATION = 0.5; //test it

    public HardwareMap hwmap = null;
    private ElapsedTime runtime = new ElapsedTime(); //idk what it does, just trying to get public methods to work


    public HardwareBigBoy() {

    }

    public void init(HardwareMap aMap) {
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
        leftSlideMotor = hwmap.dcMotor.get("leftSlideMotor");

        colorServoArm = hwmap.servo.get("colorServoArm");
        colorSensor = hwmap.colorSensor.get("color");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotor.Direction.FORWARD);//TODO Find out which are forward and which are reverse
        //Set Mode
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Power to 0
        stopMoving();
        rightServoArm.setPosition(1-SLIDE_ARM_HOME);
        leftServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);

    }


    public void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
        rightServoArm.setPosition(1-SLIDE_ARM_HOME);
        leftServoArm.setPosition(SLIDE_ARM_HOME);
        colorServoArm.setPosition(COLOR_ARM_HOME);


    }
}


