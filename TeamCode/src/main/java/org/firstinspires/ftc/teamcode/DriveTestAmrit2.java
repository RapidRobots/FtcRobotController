package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//importing the DcMotors and the OpMode


@TeleOp(name="DriveTestAmrit2")
public class DriveTestAmrit2 extends OpMode {

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

    }

    @Override
    public void loop() {


        forward = gamepad1.left_stick_y; // 0
        strafe = gamepad1.left_stick_x; // 1
        rotate = gamepad1.right_stick_x;
        armRotateSpeed = gamepad2.left_stick_y;
        armExtend = gamepad2.right_stick_y;

        rotateCheck = gamepad2.a;

        servoCheck = gamepad2.right_bumper;

        intakePressneg = gamepad2.right_trigger;

        intakePresspos = gamepad2.left_trigger;



        duckSpin = gamepad1.x;

        duckSpin2 = gamepad1.y;



        int maxPos = 400;
        int minPos = 2900;

        if (armExtendMotor.getCurrentPosition() >= maxPos && armExtend > 0) {
            armExtendMotor.setPower(0.1);
        }
        if (armExtendMotor.getCurrentPosition() < minPos && armExtend < 0) {
            armExtendMotor.setPower(-0.1);
        }



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
            leftFrontMotor.setPower(-rotate);
            rightBackMotor.setPower(-rotate);//negs

            rightFrontMotor.setPower(-rotate);//negs
            leftBackMotor.setPower(-rotate);
        }

        int y = 5;

        double x = 0.5;

        if (y == 5) {
            leftFrontMotor.setPower(x);
        }
        if (y > 5) {
            leftFrontMotor.setPower(x);
        }
        if (y < 5) {
            leftFrontMotor.setPower(x);
        }
        if (y != 5) {
            leftFrontMotor.setPower(x);
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

        if (topLeftDiagonal > 0.75)
        {
            topLeftDiagonal = 0.75;
        }
        if (topLeftDiagonal < -0.75)
        {
            topLeftDiagonal = -0.75;
        }
        if (topRightDiagonal > 0.75)
        {
            topRightDiagonal = 0.75;
        }
        if (topLeftDiagonal < -0.75)
        {
            topLeftDiagonal = -0.75;
        }

        if (intakePresspos > 0) {

            intakePower = 0.8;
        }
        if (intakePressneg > 0) {

            intakePower = -0.8;

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



        if (armRotateSpeed < -0.4) {
            armRotateSpeed = -0.4;
            armRotate = armRotateSpeed;
        }
        if (armRotateSpeed > 0.4) {
            armRotateSpeed = 0.4;
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
            armExtend = 1;
        }
        if (armExtend < 0) {
            armExtend = -1;
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
            duckPower = 0.35;
        }
        if (duckSpin != true && duckSpin2 != true) {
            duckPower = 0;
        }

        if (duckSpin2 == true) {
            duckPower = -0.35;
        }
        if (duckSpin2 != true && duckSpin != true) {
            duckPower = 0;
        }





        leftFrontMotor.setPower(topLeftDiagonal);//

        leftFrontMotor.setPower(0.5);

        rightBackMotor.setPower(-topLeftDiagonal);//

        rightFrontMotor.setPower(-topRightDiagonal);
        leftBackMotor.setPower(topRightDiagonal);



        intakeMotor.setPower(-intakePower);

        armRotateMotor.setPower(armRotateSpeed);
        armExtendMotor.setPower(armExtend);

        duck_wheel.setPower(duckPower);




    }
}
