package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBob {
    public DcMotor rightFrontMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;

    public HardwareMap hwmap = null;

    public void init(HardwareMap aMap){
        // Hardware Map assignment
        hwmap = aMap;

        //Motor assignments from config in app
        rightFrontMotor = hwmap.dcMotor.get("Right_Front_Motor");
        leftFrontMotor = hwmap.dcMotor.get("Left_Front_Motor");
        leftBackMotor = hwmap.dcMotor.get("Left_Back_Motor");
        rightBackMotor = hwmap.dcMotor.get("Right_Back_Motor");

        //Set inital mode of motor
        rightFrontMotor.setMode();
        leftFrontMotor = hwmap.dcMotor.get("Left_Front_Motor");
        leftBackMotor = hwmap.dcMotor.get("Left_Back_Motor");
        rightBackMotor = hwmap.dcMotor.get("Right_Back_Motor");

    }
}
