/*
 * Blue program for 1 side
 *
 *
 * */





package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonDriveRedSide1")
public class AutonDriveRedSide1 extends LinearOpMode {




    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    DcMotor duck_wheel = null;

    DcMotor armRotateMotor = null;
    DcMotor armExtendMotor = null;

    DcMotor intake_motor;

    ColorSensor colorSensor;

    //BNO055IMU imu;


    double topLeftDiagonal;
    double topRightDiagonal;

    double leftSpeed;
    double rightSpeed;

    boolean duckSpin;
    double duckPower;

    int position = 0;

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

    /*public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;


        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= 1) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = rightSpeed;
        }

        return onTarget;

    }








*/
    public boolean detectGreen() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        boolean greenCheck = false;


        float hsb[] = new float[3];

        Color.RGBToHSV(red, green, blue, hsb);

            /*telemetry.addData("Blue - Red", blue - red);
            telemetry.addData("Red - Blue", red - blue);
            telemetry.addData("Blue + Red/divident", greenCheck);*/



        telemetry.addData("Hue : ", hsb[0]);
        telemetry.addData("Saturation : ", hsb[1]);
        telemetry.addData("Brightness : ", hsb[2]);

        if (hsb[0] > 100 && 130 > hsb[0]) {
            telemetry.addData("This is ", "green");
            greenCheck = true;
        }
        return greenCheck;
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

        colorSensor = hardwareMap.colorSensor.get("color_sensor");


        waitForStart();

        drive(-0.2, 0, 2);


        drive(0.5, 0, 7.5);

        drive(0, 0.5, 110);//110



        //drive(-0.2, 0, 2);

        //drive(0.5, 0, 6);


        drive(0, 0.2, 35);//14

        sleep(10);

        drive(0, 0.1, 10);

        sleep(10);





        /*rotate(-20);

        sleep(150);

        rotate(0);*/




        //duck

        duck_wheel.setPower(-0.25);

        sleep(3100);

        duck_wheel.setPower(-1);

        sleep(500);

        duck_wheel.setPower(0);

        rotate(20);

        sleep(50);

        rotate(0);


        drive(0, -0.5, 86.5);






        /*rotate(20);

        sleep(150);

        rotate(0);
*/





        //place object s

        drive(0.5, 0, 7.5);

        armRotateMotor.setPower(-0.75);

        sleep(150);

        armRotateMotor.setPower(-0.2);

        armExtendMotor.setPower(0.5);



        sleep(2000);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(-0.22);

        intake_motor.setPower(0.45);

        sleep(320);

        intake_motor.setPower(0);

        armExtendMotor.setPower(-0.5);

        sleep(2000);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(0.75);

        sleep(250);

        armRotateMotor.setPower(0);


        //do the putting of an object here in place of wait



        drive(0, -0.5, 30);

        rotate(20);

        sleep(280);

        rotate(0);

        drive(0, -0.5, 55);

        drive(0.5, 0, 42.5);

        drive(0, 0.5, 65);


        stop();


    }




}
