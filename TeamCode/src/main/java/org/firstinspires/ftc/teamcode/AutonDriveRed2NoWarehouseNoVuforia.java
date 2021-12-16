/*
 * Blue program for 1 side
 *
 *
 * */





package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="AutonDriveRed2NoWareHouseNoVuforia")
public class AutonDriveRed2NoWarehouseNoVuforia extends LinearOpMode {




    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    DcMotor duck_wheel = null;

    DcMotor armRotateMotor = null;
    DcMotor armExtendMotor = null;

    DcMotor intake_motor;

    Servo testServo = null;



    //BNO055IMU imu;


    double topLeftDiagonal;
    double topRightDiagonal;


    //ModernRoboticsI2cGyro gyro = null;


    double pi = 3.1415926536;
    long seconds;
    double rpm;

    double aboslutePower;

    double COUNTS_PER_INCH;

    double runtime;








    private void drive(double forward, double strafe, double dist)
    {

        topLeftDiagonal = forward - strafe; // + =
        topRightDiagonal = forward + strafe;//- =

        leftFrontMotor.setPower(-topLeftDiagonal);
        rightBackMotor.setPower(topLeftDiagonal);//-

        rightFrontMotor.setPower(topRightDiagonal);//-
        leftBackMotor.setPower(-topRightDiagonal);

        telemetry.addData("power", topLeftDiagonal);
        telemetry.update();



        seconds = distanceCalc(dist);

        //Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        /*for (int i = 0; i < seconds + 1; i++) {
            if (onHeading(0.75, )) {
                onHeading(0.75,0,0.1);

            }
            sleep(1);
        }*/



        sleep(seconds);




        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);//-

        rightFrontMotor.setPower(0);//-
        leftBackMotor.setPower(0);





    }
    /*public void encoderDrive(double speed, double leftInches, double rightInches,double leftBackInches, double rightBackInches) {
        COUNTS_PER_INCH = (223 * 1) / (4 * 3.1415);

        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftFrontMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightFrontMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            leftFrontMotor.setTargetPosition(newLeftTarget);
            rightFrontMotor.setTargetPosition(-newRightTarget);
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightBackMotor.setTargetPosition(-newRightBackTarget);


            // Turn On RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            leftFrontMotor.setPower(speed);
            rightFrontMotor.setPower(-speed);
            leftBackMotor.setPower(speed);
            rightBackMotor.setPower(-speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontMotor.getCurrentPosition(),
                        rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            /*leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
    //  sleep(250);   // optional pause after each move */



    private void rotate(double rotate)
    {
        leftFrontMotor.setPower(-rotate);//
        rightBackMotor.setPower(-rotate);//

        rightFrontMotor.setPower(-rotate);
        leftBackMotor.setPower(-rotate);
    }


    public long distanceCalc(double distanceInches) {
        aboslutePower = topLeftDiagonal;

        if (aboslutePower < 0) {
            aboslutePower = -1 * aboslutePower;
        }

        rpm = 312 * aboslutePower;
        rpm = rpm/60000;

        seconds = (long) (distanceInches / (rpm * 4.8 * pi));
        //in inches

        telemetry.addData("Seconds waited", seconds);
        telemetry.update();

        //radius = 4.8cm
        return seconds;

    }



    @Override
    public void runOpMode()
    {

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        /*parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

*/
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("imu");


        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_rear");
        rightBackMotor = hardwareMap.dcMotor.get("right_rear");



        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        armRotateMotor = hardwareMap.dcMotor.get("arm_rotate");
        armExtendMotor = hardwareMap.dcMotor.get("arm_extend");

        intake_motor = hardwareMap.dcMotor.get("intake_motor");

        testServo = hardwareMap.servo.get("test_servo");



        drive(0.5,0,8);

        drive(0,0.5,45);

        sleep(50);

        drive(0, 0.25, 10);





        //duck start

        duck_wheel.setPower(-0.25);

        sleep(2600);

        duck_wheel.setPower(-1);

        sleep(500);

        duck_wheel.setPower(0);

        //duck end

        rotate(20);

        sleep(280);

        rotate(0);

        //align for hub

        drive(0.75,0,35);

        //place object

        double pos = 0.5;

        testServo.setPosition(pos);

        armRotateMotor.setPower(-1);

        sleep(200);

        armRotateMotor.setPower(0);

        armExtendMotor.setPower(0.5);



        sleep(2000);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(0);

        intake_motor.setPower(0.55);

        sleep(520);

        intake_motor.setPower(0);

        armExtendMotor.setPower(-0.5);

        sleep(2000);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(0.75);

        sleep(250);

        armRotateMotor.setPower(0);

        //end of place object

        drive(-0.25, 0.5, 30);

        stop();


    }




}
