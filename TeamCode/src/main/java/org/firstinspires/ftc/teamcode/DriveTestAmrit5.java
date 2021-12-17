package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

//importing the DcMotors and the OpMode


@TeleOp(name="DriveTestAmrit5")
public class DriveTestAmrit5 extends OpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor intakeMotor = null;

    DcMotor armRotateMotor = null;
    DcMotor armExtendMotor = null;

    DcMotor duck_wheel = null;

    DcMotor intake_motor;

    CRServo armServo;

    DigitalChannel touch_extend;

    DigitalChannel touch_retract;

    boolean capPressed;
    double capSpeed;

    double topLeftDiagonal;
    double topRightDiagonal;
    double forward;
    double strafe;
    double rotate;
    double armRotate;
    double armExtend;
    double servoPower = 1;

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






    public DistanceSensor distance_sensor;


    boolean speedCapButton;






    //setting up the DcMotors



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

        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        intake_motor = hardwareMap.dcMotor.get("intake_motor");

        touch_extend = hardwareMap.get(DigitalChannel.class, "sensor_digital1");
        touch_retract = hardwareMap.get(DigitalChannel.class, "sensor_digital3");

        capPressed = false;
        capSpeed = 0.75;

    }

    @Override
    public void loop() {

        if(speedCapButton == false) {
            capPressed = false;
        }


        forward = gamepad1.left_stick_y; // 0
        strafe = gamepad1.left_stick_x; // 1
        rotate = gamepad1.right_stick_x;
        armRotateSpeed = gamepad2.left_stick_y;
        armExtend = gamepad2.right_stick_y;

        touchExtend = touch_extend.getState();
        touchRetract = touch_retract.getState();


        rotateCheck = gamepad2.a;

        servoCheck = gamepad2.right_bumper;

        intakePressneg = gamepad2.right_trigger;

        intakePresspos = gamepad2.left_trigger;



        duckSpin = gamepad1.x;

        duckSpin2 = gamepad1.b;



        speedCapButton = gamepad1.left_bumper;

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
            leftFrontMotor.setPower(-rotate);
            rightBackMotor.setPower(-rotate);//negs

            rightFrontMotor.setPower(-rotate);//negs
            leftBackMotor.setPower(-rotate);
        }
        if (rotate < 0) {
            if (rotate < -0.75) {
                rotate = -0.75;
            }
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
        telemetry.addData("TopLeftDiagonal",topLeftDiagonal);
        telemetry.addData("TopRightDiagonal",topRightDiagonal);

        if (intakePresspos > 0) {

            intakePower = 1;
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



        if (armRotateSpeed < -0.5) {
            armRotateSpeed = -0.5;
            armRotate = armRotateSpeed;
        }
        if (armRotateSpeed > 0.5) {
            armRotateSpeed = 0.5;
            armRotate = armRotateSpeed;

        }
        armRotate = armRotateSpeed;
        if (rotateCheck == true && rotateCheckPress == true) {
            armRotateStore = 0;
            rotateCheckPress = false;
        }
        if (rotateCheck == true && rotateCheckPress == false) {
            armRotateStore = armRotateSpeed;
            rotateCheckPress = true;
        }





        if (armExtend > 0) {
            if (touchExtend == true) {
                armExtend = 0.8;
            }
            if (touchExtend == false) {
                armExtend = 0;
                telemetry.addData("Extend", armExtend);

            }
        }
        if (armExtend < 0) {
            if (touchRetract == true) {
                armExtend = -0.8;
            }
            if (touchRetract == false) {
                armExtend = 0;
                telemetry.addData("Retract", armExtend);

            }
        }

        if (touchExtend == true && touchRetract == true) {
            telemetry.addData("Moving", armExtend);

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
            duckPower = 0.5;
        }
        if (duckSpin != true && duckSpin2 != true) {
            duckPower = 0;
        }

        if (duckSpin2 == true) {
            duckPower = -0.5;
        }
        if (duckSpin2 != true && duckSpin != true) {
            duckPower = 0;
        }

        if (gamepad2.a == true) {
            armRotateSpeed = -0.32;
            telemetry.addData("Held speed", armRotateSpeed);
            telemetry.update();
        }





        leftFrontMotor.setPower(topLeftDiagonal);//
        rightBackMotor.setPower(-topLeftDiagonal);//

        rightFrontMotor.setPower(-topRightDiagonal);
        leftBackMotor.setPower(topRightDiagonal);



        intakeMotor.setPower(-intakePower);

        armRotateMotor.setPower(armRotateSpeed);
        armExtendMotor.setPower(armExtend);

        duck_wheel.setPower(duckPower);

        telemetry.addData("Arm Movement Speed", armRotateSpeed);
        telemetry.update();




    }
}
