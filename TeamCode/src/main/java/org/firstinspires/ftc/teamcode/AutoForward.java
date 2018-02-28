package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto-Forward", group="Pushbot")
public class AutoForward extends LinearOpMode {
    private BiggerBoyHardware robot = new BiggerBoyHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Bigger Boy Powered");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        driveStraight(4);
        driveBackwards(.5);

        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);
        idle();

    }

    public void driveStraight(double feet){
        telemetry.addData("Motion", "Driving Straight");
        telemetry.update();
        double rotations = feet / BiggerBoyHardware.WHEELS_CIRCUM;
        double ticks = rotations * BiggerBoyHardware.TICKS_PER_ROTATION;
        resetEncoders();
        setRunToPosition();

        robot.leftFrontMotor.setTargetPosition(robot.leftFrontMotor.getCurrentPosition() + (int)ticks);
        robot.leftBackMotor.setTargetPosition(robot.leftBackMotor.getCurrentPosition() + (int)ticks);
        robot.rightFrontMotor.setTargetPosition(robot.rightFrontMotor.getCurrentPosition() + (int)ticks);
        robot.rightBackMotor.setTargetPosition(robot.rightBackMotor.getCurrentPosition() + (int)ticks);


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
        setRunToPosition();

        robot.leftFrontMotor.setTargetPosition((int) -ticks);
        robot.leftBackMotor.setTargetPosition((int) -ticks);
        robot.rightFrontMotor.setTargetPosition((int) -ticks);
        robot.rightBackMotor.setTargetPosition((int) -ticks);



        robot.leftFrontMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.leftBackMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.rightFrontMotor.setPower(BiggerBoyHardware.TURN_SPEED);
        robot.rightBackMotor.setPower(BiggerBoyHardware.TURN_SPEED);
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
        int countBusy = 4;
        while (countBusy > 2 && runtime.seconds() < 5){
            countBusy = 0;
            if(robot.leftFrontMotor.isBusy())
                countBusy++;
            if(robot.leftBackMotor.isBusy())
                countBusy++;
            if(robot.rightBackMotor.isBusy())
                countBusy++;
            if(robot.rightFrontMotor.isBusy())
                countBusy++;
        }
    }

}

