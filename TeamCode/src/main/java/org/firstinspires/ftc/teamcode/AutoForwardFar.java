package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto-Forward-Far", group="Pushbot")
public class AutoForwardFar extends LinearOpMode {
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
        driveStraight(6);
        driveBackwards(.5);

        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);
        idle();

    }

    public void driveBackwards(double feet) {

        double rotations = feet / robot.WHEELS_CIRCUM;
        double ticks = rotations * robot.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition((int) ticks);
        robot.leftBackMotor.setTargetPosition((int) ticks);
        robot.rightFrontMotor.setTargetPosition((int) ticks);
        robot.rightBackMotor.setTargetPosition((int) ticks);

        setRunToPosition();


        robot.leftFrontMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.leftBackMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.rightFrontMotor.setPower(-1*robot.DRIVE_SPEED);
        robot.rightBackMotor.setPower(-1*robot.DRIVE_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();
    }


    public void driveStraight(double feet){

        double rotations = feet /robot.WHEELS_CIRCUM;
        double ticks = rotations * robot.TICKS_PER_ROTATION;
        resetEncoders();

        robot.leftFrontMotor.setTargetPosition(-1*(int)ticks);
        robot.leftBackMotor.setTargetPosition(-1*(int)ticks);
        robot.rightFrontMotor.setTargetPosition(-1*(int)ticks);
        robot.rightBackMotor.setTargetPosition(-1*(int)ticks);

        setRunToPosition();

        robot.leftFrontMotor.setPower(robot.DRIVE_SPEED);
        robot.leftBackMotor.setPower(robot.DRIVE_SPEED);
        robot.rightFrontMotor.setPower(robot.DRIVE_SPEED);
        robot.rightBackMotor.setPower(robot.DRIVE_SPEED);
        whileIsBusy();
        robot.stopMoving();
        resetEncoders();

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
    public void whileIsBusy() {
        while (robot.leftFrontMotor.isBusy()|| robot.leftBackMotor.isBusy()||
                robot.rightFrontMotor.isBusy() || robot.rightBackMotor.isBusy()){}
    }
}

