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

        //Set Mode

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Power to 0
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);



    }
}
