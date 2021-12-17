package org.firstinspires.ftc.teamcode;



import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//importing the DcMotors and the OpMode

//100% Amrit's Code


@TeleOp(name="DriveTestAmrit4")
public class DriveTestAmrit4 extends OpMode {

    int duckCounter;

    boolean capPressed;

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor intakeMotor = null;

    double capSpeed;

    DcMotor armRotateMotor = null;
    DcMotor armExtendMotor = null;

    DcMotor duck_wheel = null;

    DcMotor intake_motor;

    CRServo armServo;


    DigitalChannel touch_arm_retract, touch_arm_extend;



    double topLeftDiagonal;
    double topRightDiagonal;
    double forward;
    double strafe;
    double rotate;
    double armRotate;
    double armExtend;
    double servoPower = 1;

    double duckSpeed;

    double armRotateStore;

    double armRotateSpeed;

    boolean duckSpin;
    boolean duckSpin2;
    double duckPower;


    double intakePower;

    boolean servoCheck;

    boolean servoStop;

    boolean rotateCheck;

    boolean rotateCheckPress;

    double intakePresspos;

    double intakePressneg;

    boolean touchExtend;
    boolean touchRetract;

    ColorSensor colorSensor;
    ColorSensor colorSensorBlue;

    boolean greenCheck;
    boolean greenCheckBlue;






    DistanceSensor armRotatePosition;


    boolean speedCapButton;

    float hsb[] = new float[3];
    float hsb2[] = new float[3];

    double colorDist;
    double colorDistBlue;








    //setting up the DcMotors



    public void duckWheelForLoop() {
        for (int sleepCounter = 0; sleepCounter <= 500; sleepCounter++) {
            duck_wheel.setPower(duckSpeed);
            if (sleepCounter == 400) {
                duckSpeed = 0.75;
            }
        }
        duckSpeed = 0.4;
    }
    public void duckWheelSleepLoop(int sleepTime) {
        for (int sleepCounter = 0; sleepCounter <= sleepTime; sleepCounter++) {

        }

    }


    public void duckWheel() {
        for(int i = 0; i < 11; i++) {
            duckWheelForLoop();
            duckWheelSleepLoop(550);
        }




    }

    public void detectGreen() {

        greenCheck = false;
        greenCheckBlue = false;



        Color.RGBToHSV(colorSensor.red(), colorSensor.blue(), colorSensor.green(),hsb);
        Color.RGBToHSV(colorSensorBlue.red(), colorSensorBlue.blue(), colorSensorBlue.green(),hsb2);

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
        if (hsb2[0] > 215 && 235 > hsb2[0]) {
            telemetry.addData("This is ", "green");
            greenCheckBlue = true;
        }
        if (colorDistBlue < 5) {
            greenCheckBlue = true;
        }

    }

    @Override
    public void init()
    {
        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_rear");
        rightBackMotor = hardwareMap.dcMotor.get("right_rear");
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        armRotateMotor = hardwareMap.dcMotor.get("arm_rotate");
        armExtendMotor = hardwareMap.dcMotor.get("arm_extend");
        armServo = hardwareMap.crservo.get("arm_servo");

        armRotatePosition = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        intake_motor = hardwareMap.dcMotor.get("intake_motor");

        touch_arm_retract = hardwareMap.get(DigitalChannel.class, "sensor_digital3");
        touch_arm_extend = hardwareMap.get(DigitalChannel.class, "sensor_digital1");

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        colorSensorBlue = hardwareMap.colorSensor.get("color_sensor_blue");

        /* set the digital channel to input. */
        touch_arm_retract.setMode(DigitalChannel.Mode.INPUT);
        touch_arm_extend.setMode(DigitalChannel.Mode.INPUT);

        //touch_top = hardwareMap.touchSensor.get("touch_top_arm");

        capPressed = false;
        capSpeed = 0.75;




        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.




    }

    @Override
    public void loop() {

        colorDist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        colorDistBlue = ((DistanceSensor) colorSensorBlue).getDistance(DistanceUnit.CM);

        Color.RGBToHSV(colorSensor.red(), colorSensor.blue(), colorSensor.green(),hsb);
        Color.RGBToHSV(colorSensorBlue.red(), colorSensorBlue.blue(), colorSensorBlue.green(),hsb2);

        detectGreen();

        telemetry.addData("range", armRotatePosition.getDistance(DistanceUnit.MM));
        telemetry.addData("color sensor red color", hsb[0]);
        telemetry.addData("color sensor blue color", hsb2[0]);
        telemetry.addData("color sensor red dist", colorDist);
        telemetry.addData("color sensor blue dist", colorDistBlue);

        telemetry.addData("is green?", greenCheck);
        telemetry.addData("is green blue?", greenCheckBlue);

        telemetry.addData("touch retract", touch_arm_retract.getState());
        telemetry.addData("touch extend", touch_arm_extend.getState());



        if(speedCapButton == false) {
            capPressed = false;
        }


        forward = gamepad1.left_stick_y; // 0
        strafe = gamepad1.left_stick_x; // 1
        rotate = gamepad1.right_stick_x;
        armRotateSpeed = gamepad2.left_stick_y;
        armExtend = gamepad2.right_stick_y;



        speedCapButton = gamepad1.left_bumper;


        rotateCheck = gamepad2.a;

        servoCheck = gamepad2.right_bumper;

        intakePressneg = gamepad2.right_trigger;

        intakePresspos = gamepad2.left_trigger;



        duckSpin = gamepad1.x;

        duckSpin2 = gamepad1.b;



        speedCapButton = gamepad1.left_bumper;
        //intakeCheck = gamepad1.a;

        //Button X is ducky

        /*if (forward < 0 && objectDetect == true) {
            forward = 0;
        }*/

        intakePower = gamepad1.right_trigger;

        topLeftDiagonal = forward - strafe; // + =
        topRightDiagonal = forward + strafe;//- =

        /*if (speedCapButton) {
            if (speedCap == 0.75 && changed != true) {
                speedCap = 0.5;
                 changed = true;
            }
            if (speedCap == 0.5 && changed != true) {
                speedCap = 0.75;
            }



        }*/



        if (rotate > 0) {

            if (rotate > 0.75) {
                rotate = 0.75;
            }
            telemetry.addData("rotate : ", -1*rotate);
            leftFrontMotor.setPower(-rotate);
            rightBackMotor.setPower(-rotate);//negs

            rightFrontMotor.setPower(-rotate);//negs
            leftBackMotor.setPower(-rotate);
        }
        if (rotate < 0) {
            if (rotate < -0.75) {
                rotate = -0.75;
            }
            telemetry.addData("rotate : ", -1*rotate);
            leftFrontMotor.setPower(-rotate);
            rightBackMotor.setPower(-rotate);//negs

            rightFrontMotor.setPower(-rotate);//negs
            leftBackMotor.setPower(-rotate);
        }

        if(speedCapButton == true && capSpeed == 0.25 && capPressed == false) {
            capPressed = true;
            capSpeed = 0.75;
            telemetry.addData("Speed : ", capSpeed);


        }

        if(speedCapButton == true && capSpeed == 0.75 && capPressed == false) {
            capPressed = true;
            capSpeed = 0.25;
            telemetry.addData("Speed : ", capSpeed);

        }

        telemetry.addData("Speed : ", capSpeed);

        if (topLeftDiagonal > capSpeed)
        {
            topLeftDiagonal = capSpeed;
            telemetry.addData("TopLeftDiagonal",topLeftDiagonal);
        }
        if (topLeftDiagonal < -capSpeed)
        {
            topLeftDiagonal = -capSpeed;
        }
        if (topRightDiagonal > capSpeed)
        {
            topRightDiagonal = capSpeed;
        }
        if (topRightDiagonal < -capSpeed)
        {
            topRightDiagonal = -capSpeed;
        }

        if (intakePresspos > 0) {

            intakePower = 0.9;
        }
        if (intakePressneg > 0) {

            intakePower = -0.38;

        }

        if (intakePresspos > 0 && intakePressneg > 0) {
            intakePower = 0;
        }

        if (intakePower > 1) {
            intakePower = 1;
        }

        if (intakePower < -1) {
            intakePower = -1;
        }



        if (armRotateSpeed < -0.7) {
            armRotateSpeed = -0.7;
            armRotate = armRotateSpeed;
        }
        if (armRotateSpeed > 0.7) {
            armRotateSpeed = 0.7;
            armRotate = armRotateSpeed;

        }
        //armRotate = armRotateSpeed;
        if (rotateCheck == true && rotateCheckPress == true) {
            armRotateStore = 0;
            rotateCheckPress = false;
        }
        if (rotateCheck == true && rotateCheckPress == false) {
            armRotateStore = armRotateSpeed;
            rotateCheckPress = true;
        }





        if (armExtend > 0.05) {

            /*if (touch_arm_extend.getState() == false)
            {
                telemetry.addData("Touch Sensor:", "Arm extend Touched");
                /* The below power helps the motor to stay the arm where it is currently
                armExtend = 0;
            }
            if (touch_arm_extend.getState() == true)
            {*/

                /* The below power helps the motor to stay the arm where it is currently */
                armExtend = 0.75;
           // }
        }
        if (armExtend < 0.05) {
            /*if (touch_arm_retract.getState() == false)
            {
                telemetry.addData("Touch Sensor:", "Arm retract Touched");
                /* The below power helps the motor to stay the arm where it is currently
                armExtend = 0;
            }
            if (touch_arm_retract.getState() == true)
            {*/

                /* The below power helps the motor to stay the arm where it is currently */
                armExtend = -0.75;
            //}




        }




        /*if (servoCheck == true) {

            armServo.setPower(0.75);
            telemetry.addData("Servo Position", armServo.getPower());

            telemetry.update();
        }
        if (servoCheck == false) {


            //servoPower = armServo.getPower();
            //armServo.setPower(0);
            telemetry.addData("Servo Position", servoPower);
            telemetry.update();
        }*/


        if (duckSpin == true) {
            duckPower = 0.52;

            telemetry.addData("Duck Speed", duckPower);
        }
        if (duckSpin != true && duckSpin2 != true) {
            duckPower = 0;
            duckCounter = 0;
        }

        if (duckSpin2 == true) {
            duckPower = -0.52;

            telemetry.addData("Duck Speed", duckPower);
        }
        if (duckSpin2 != true && duckSpin != true) {
            duckPower = 0;
            duckCounter = 0;
        }

        if (gamepad2.a == true) {
            armRotateSpeed = -0.32;
            telemetry.addData("Held speed", armRotateSpeed);
            telemetry.update();
        }






        /*if (touch_extend.getState() == true) {
            armRotateSpeed = -0.32;
            telemetry.addData("using touch : ", "sensor arm rotate");
            telemetry.update();
        }*/






        leftFrontMotor.setPower(topLeftDiagonal);
        rightBackMotor.setPower(-topLeftDiagonal);//-

        rightFrontMotor.setPower(-topRightDiagonal);//-
        leftBackMotor.setPower(topRightDiagonal);





        intakeMotor.setPower(-intakePower);

        armRotateMotor.setPower(armRotateSpeed);
        armExtendMotor.setPower(armExtend);

        duck_wheel.setPower(duckPower);

        telemetry.addData("Arm Movement Speed", armRotateSpeed);
        telemetry.update();




    }
}
