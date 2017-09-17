package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareBob {
    public DcMotor rightFrontMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;

    public HardwareMap hwmap = null;
    private ElapsedTime runtime = new ElapsedTime(); //idk what it does, just trying to get public methods to work
    

    public HardwareBob() {
    
    }

    public void init(HardwareMap aMap){
        // Hardware Map assignment
        hwmap = aMap;

        //Motor assignments from config in app
        rightFrontMotor = hwmap.dcMotor.get("Right_Front_Motor");
        leftFrontMotor = hwmap.dcMotor.get("Left_Front_Motor");
        leftBackMotor = hwmap.dcMotor.get("Left_Back_Motor");
        rightBackMotor = hwmap.dcMotor.get("Right_Back_Motor");

        //Set Mode
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Power to 0
        stopMoving();

    }

/* Braha Testing Zone */
    static final double     FORWARD_SPEED = 0.6;
    static final double     LEFT_MOTOR_OFFSET = 0.0; //Probably > 0 because robot moves left when going straight
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public void stopMoving() {
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
    }
    public void driveStraight(double x) throws InterruptedException {
        leftFrontMotor.setPower((DRIVE_SPEED + LEFT_MOTOR_OFFSET));
        leftBackMotor.setPower((DRIVE_SPEED + LEFT_MOTOR_OFFSET));
        rightFrontMotor.setPower(DRIVE_SPEED);
        rightBackMotor.setPower(DRIVE_SPEED);
        runtime.reset();
        //TODO Does this code have to be in the opMode?
        /*while (opModeIsActive() && (runtime.seconds() < x)) {
            idle();
        }*/
        while ((runtime.seconds() < x)) {
            wait();
        }
    }
}
