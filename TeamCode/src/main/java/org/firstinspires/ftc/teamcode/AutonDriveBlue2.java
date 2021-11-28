/*
 * Blue program for 1 side
 *
 *
 * */





package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonDriveBlue2")
public class AutonDriveBlue2 extends LinearOpMode {




    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    DcMotor duck_wheel = null;

    DcMotor armRotateMotor = null;
    DcMotor armExtendMotor = null;

    DcMotor intake_motor;

    //BNO055IMU imu;


    double topLeftDiagonal;
    double topRightDiagonal;

    double leftSpeed;
    double rightSpeed;

    boolean duckSpin;
    double duckPower;

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


    public long distanceCalc(double distance) {
        aboslutePower = topLeftDiagonal;

        if (aboslutePower < 0) {
            aboslutePower = -1 * aboslutePower;
        }

        rpm = 223 * aboslutePower;
        rpm = rpm/60000;

        seconds = (long) (distance / (rpm * 4.8 * 2 * pi));

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


        waitForStart();


        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        drive(0, 0.5,15);

        drive(0, -0.5, 20); //30

        sleep(10);

        //rotate(20);

        //sleep(5);

        //rotate(0);



        drive(-0.5, 0, 34);

        //drive(-0.5, 0, 30);

        sleep(10);


        drive(-0.5, 0, 2);

        drive(0, 0.5, 4);

        sleep(10);

        drive(-0.1, 0, 4);




        //duck

        duck_wheel.setPower(0.25);

        sleep(2600);

        duck_wheel.setPower(1);

        sleep(200);

        duck_wheel.setPower(0);




        drive(0, -0.5, 15);

        drive(0.5, 0, 65);

        //place object

        rotate(20);

        sleep(220);

        rotate(0);

        drive(0.5, 0, 5);

        armRotateMotor.setPower(-0.75);

        sleep(250);

        armRotateMotor.setPower(0);

        armExtendMotor.setPower(0.5);



        sleep(2000);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(0);

        intake_motor.setPower(0.55);

        sleep(320);

        intake_motor.setPower(0);

        armExtendMotor.setPower(-0.5);

        sleep(2000);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(0.75);

        sleep(250);

        armRotateMotor.setPower(0);


        //do the putting of an object here in place of wait

        rotate(-20);

        sleep(220);

        rotate(0);

        drive(0.5, 0, 30);

        drive(0, 0.5, 120);

        drive(0.5, 0, 100);

        sleep(100);

        drive(0.75, 0, 20);



        /*Go to alliance shipping hub
        Go to the wall on the left
        go to duck
        spin duck
        go forward a bit
        strafe to wall
        go forward into warehouse

         */

        //Spin Ducky
/*
        sleep(2500);

        drive(0, 0.5);

        sleep(1000);

        drive(0.5, 0);

        sleep(2000);

        drive(0, -0.5);

        sleep(1000);

        drive(0.5, 0);

        sleep(2000);

        drive(0,0.5);

        sleep(1000);

        drive(0.5,0);
*/
        stop();


    }




}
