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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

@Autonomous(name="AutonDriveBlue1ColorNoDuck")
public class AutonDriveBlue1ColorNoDuck extends LinearOpMode {




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

    int position;


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




    public boolean detectGreen() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        double colorDist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        boolean greenCheck = false;


        float hsb[] = new float[3];

        Color.RGBToHSV(red, green, blue, hsb);

            /*telemetry.addData("Blue - Red", blue - red);
            telemetry.addData("Red - Blue", red - blue);
            telemetry.addData("Blue + Red/divident", greenCheck);*/



        telemetry.addData("Hue : ", hsb[0]);
        telemetry.addData("Saturation : ", hsb[1]);
        telemetry.addData("Brightness : ", hsb[2]);

        telemetry.addData("Dist", colorDist);

        if (hsb[0] > 215 && 235 > hsb[0]) {
            telemetry.addData("This is ", "green");
            greenCheck = true;
        }
        if (colorDist < 5) {
            greenCheck = true;
        }
        return greenCheck;
    }

    private void drive(double forward, double strafe, double dist)
    {

        topLeftDiagonal = forward - strafe; // + =
        topRightDiagonal = forward + strafe;//- =

        leftFrontMotor.setPower(-topLeftDiagonal);
        rightBackMotor.setPower(topLeftDiagonal);//-

        rightFrontMotor.setPower(topRightDiagonal);//-
        leftBackMotor.setPower(-topRightDiagonal);



        seconds = distanceCalc(dist, topLeftDiagonal, topRightDiagonal);

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


    public long distanceCalc(double distance, double topLeftDiagonal, double topRightDiagonal) {
        aboslutePower = (abs(topLeftDiagonal) + abs(topRightDiagonal))/2;
        telemetry.addData("Left", topLeftDiagonal);
        telemetry.addData("Right", topRightDiagonal);
        telemetry.addData("Aboslute", aboslutePower);
        telemetry.update();

        if (aboslutePower < 0) {
            aboslutePower = -1 * aboslutePower;
        }

        rpm = 312 * aboslutePower;
        rpm = rpm/60000;

        seconds = ((long) (distance / (rpm * 4.8 * pi)));





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
    public void thirdLevel() {
        armRotateMotor.setPower(-0.75);

        sleep(650);//550

        armRotateMotor.setPower(-0.32);

        armExtendMotor.setPower(0.5);



        sleep(2000);

        armExtendMotor.setPower(0);


        intake_motor.setPower(0.55);

        sleep(320);

        intake_motor.setPower(0);

        intake_motor.setPower(-0.3);

        sleep(50);

        intake_motor.setPower(0.55);

        sleep(100);

        intake_motor.setPower(0);

        armExtendMotor.setPower(-0.3);

        sleep(2500);

        armExtendMotor.setPower(0);

        armRotateMotor.setPower(0.75);

        sleep(450);

        armRotateMotor.setPower(0);
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

        colorSensor = hardwareMap.colorSensor.get("color_sensor_blue");


        waitForStart();


        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);





        drive(0, -0.5, 40.5); //39

        drive(0.2, 0, 10);


        sleep(20);

        for(int i = 1; i < 20  && opModeIsActive(); i++) {//18

            telemetry.addData("Starting color detection : ", "detected : 1 Ishaan");
            telemetry.update();



            drive(-0.16, 0, 1);

            sleep(10);

            if (detectGreen()) {
                if(0 < i && i < 7) {
                    position = 1;


                }
                if(6 < i && i < 14) {
                    position = 2;

                }
                if(13 < i && i < 19) {
                    position = 3;

                }

                if (position == 0) {
                    position = 1;
                    telemetry.addData("Not found", position);
                    telemetry.update();
                }
            }




        }

        //rotate(20);

        //sleep(5);

        //rotate(0);






        telemetry.addData("Position", position);

        telemetry.update();


        drive(0, 0.5, 17);

        rotate(0.5);

        sleep(120);

        rotate(0);

        drive(0, 0.5, 12);

        drive(0.5, 0, 7.5);

        //arm

        drive(-0.5, 0, 33);

        drive(0.5, 0, 10);

        rotate(-0.5);

        sleep(120);

        rotate(0);

        drive(0, 0.5, 10);

        drive(0.5, 0, 50);


        stop();


    }




}
