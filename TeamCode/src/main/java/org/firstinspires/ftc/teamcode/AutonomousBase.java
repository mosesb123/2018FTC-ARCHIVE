
package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.robotcontroller.external.samples;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
        import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name="Pushbot: autoBase", group="Pushbot")
public class AutonomousBase extends LinearOpMode {

    /* Declare OpMode boys. */...............0
    HardwareBigBoy robot = new HardwareBigBoy();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    static final double COLOR_ARM_ANGLE = .3; //need to test
    private String teamColor = ""; //our teams color, 2 dif autos


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Big Boy Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        colorActions();
//        cryptoActions(); //unfinished

        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);
        idle();
    }


    private void turnLeft(double x) throws InterruptedException {
        robot.leftFrontMotor.setPower(-(robot.TURN_SPEED + robot.LEFT_MOTOR_OFFSET));
        robot.leftBackMotor.setPower(-(robot.TURN_SPEED + robot.LEFT_MOTOR_OFFSET));
        robot.rightFrontMotor.setPower(robot.TURN_SPEED);
        robot.rightBackMotor.setPower(robot.TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < x)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
        robot.stopMoving();
    }

    private void colorActions() throws InterruptedException { //this all assumes that teamColor == our teams color and the color sensor is in the same direction that forward drive is
        robot.colorServoArm.setPosition(COLOR_ARM_ANGLE);
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        double trueColor = red - blue;
        if (teamColor.compareTo("blue") == 0 && trueColor < 0)
            robot.driveStB();
        else if (teamColor.compareTo("red") == 0 && trueColor > 0)
            robot.driveBtS();
        robot.stopMoving();
    }
    private void cryptoActions() throws InterruptedException { //finished, none of the other functions are written though
        int picture = imageSense(); //EASY DOGGY
           if (picture == 1)
//            leftKey(); // drive to put it in the left
        else if (picture == 2)
//            middleKey(); // drive to put it in the middle
        else
//            rightKey(); //drive to put it in the right
        robot.stopMoving();
    }
    public int imageSense() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXEsTGf/////AAAAGbF6lsrAgkrrmU3OaMt7gcc7l46IpUxtcXzsdAYiAx7YESYV/QxSwRN72H5y9jgaCjE4lXFjk0K6a6n80oMQhOJ1/siCcfgrEJ1fmI6IHZPm/VAxGi29eLo1ItkuAhpi5apmatTnCamd1be54REtj10OOKPNO2W+ww7UjA23++9Rb55mtU+xRBO2wQd91ugpl6VmkUaQ3cw5YDbqc0v06cmALmoy1x4d6agXpSXDRLm6V1V+r3GYo9g1LdNiB6zSwb+dIwU6e3P8dl9iVGDM3HrBPbf/M/wmEDFEiYEOXa7nQspunnfJKEHckUJU7+qMWqddM9TBpFNLO+ExQK0rAA40plID4wZ9F83qsYh5pCcS";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("VuMark", "%s visible", vuMark);

        if (vuMark == RelicRecoveryVuMark.LEFT)
            return 1;
        if (vuMark == RelicRecoveryVuMark.CENTER)
            return 2;
        if (vuMark == RelicRecoveryVuMark.RIGHT)
            return 3;

    }
    }
