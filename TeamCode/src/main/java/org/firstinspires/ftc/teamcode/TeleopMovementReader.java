package org.firstinspires.ftc.teamcode;



import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

//importing the DcMotors and the OpMode


@TeleOp(name="TeleopMovementReader")
public class TeleopMovementReader extends OpMode {

    List<String> motorValues = new ArrayList<>();


    int counter;

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



    File myObj = new File("FIRST/data","TeleopMovementData.txt");

    FileWriter myWriter =  new FileWriter("FIRST/data/TeleopMovementData.txt");



    double topLeftDiagonal;
    double topRightDiagonal;
    double forward;
    double strafe;
    double rotate;
    double armRotate;
    double armExtend;
    double servoPower = 1;

    double aboslutePower;

    double seconds;

    double armRotateStore;

    double armRotateSpeed;

    boolean duckSpin;
    boolean duckSpin2;
    double duckPower;

    double rpm;


    double intakePower;

    boolean servoCheck;

    boolean servoStop;

    boolean rotateCheck;

    boolean rotateCheckPress;

    double intakePresspos;

    double intakePressneg;






    public DistanceSensor distance_sensor;


    boolean speedCapButton;

    public TeleopMovementReader() throws IOException {
    }


    //setting up the DcMotors



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








        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);//-

        rightFrontMotor.setPower(0);//-
        leftBackMotor.setPower(0);





    }

    public long distanceCalc(double distanceInches) {
        aboslutePower = topLeftDiagonal;

        if (aboslutePower < 0) {
            aboslutePower = -1 * aboslutePower;
        }

        rpm = 435 * aboslutePower;
        rpm = rpm/60000;

        seconds = (long) (distanceInches / (rpm * 3.77952755906 * 3.1415));
        //in inches

        telemetry.addData("Seconds waited", seconds);
        telemetry.update();

        //radius = 4.8cm
        return (long) seconds;

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

        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        intake_motor = hardwareMap.dcMotor.get("intake_motor");

        try {
            myObj.createNewFile();
        } catch (IOException e) {
            telemetry.addData("error", e);
        }


        try {

            if (myObj.createNewFile()) {
                telemetry.addData("created!", "_");
                telemetry.update();
            } else {
                telemetry.addData("exists", "-");
                telemetry.update();
            }
        } catch (IOException e) {
            telemetry.addData("error", e);
            telemetry.update();
        }



    }

    @Override
    public void loop() {




        forward = gamepad1.left_stick_y; // 0
        strafe = gamepad1.left_stick_x; // 1
        rotate = gamepad1.right_stick_x;





        topLeftDiagonal = forward - strafe; // + =
        topRightDiagonal = forward + strafe;//- =


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

            intakePower = 0.5;
        }
        if (intakePressneg > 0) {

            intakePower = -0.5;

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



        if (armRotateSpeed < -0.3) {
            armRotateSpeed = -0.3;
            armRotate = armRotateSpeed;
        }
        if (armRotateSpeed > 0.3) {
            armRotateSpeed = 0.3;
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
            armExtend = 0.4;
        }
        if (armExtend < 0) {
            armExtend = -0.4;
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
            duckPower = 0.55;
        }
        if (duckSpin != true) {
            duckPower = 0;
        }

        if (duckSpin2 == true) {
            duckPower = -0.55;
        }
        if (duckSpin2 != true) {
            duckPower = 0;
        }





        leftFrontMotor.setPower(topLeftDiagonal);//
        rightBackMotor.setPower(-topLeftDiagonal);//

        rightFrontMotor.setPower(-topRightDiagonal);
        leftBackMotor.setPower(topRightDiagonal);




        counter +=1;

        if (counter < 5) {
            motorValues.add("_");
            motorValues.add("-");
            motorValues.add("_");
            motorValues.add("/");

        }

        if (counter == 5) {
            motorValues.add(toString().valueOf(topRightDiagonal));
            motorValues.add("-");
            motorValues.add(toString().valueOf(topLeftDiagonal));

            motorValues.add("/");

            counter = 0;
        }



        try {

            myWriter.write(String.valueOf(motorValues));

            telemetry.addData("Done", "-");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addData("error", "-");
            telemetry.update();
            e.printStackTrace();
        }



        telemetry.addData("Motor Values", motorValues);
        telemetry.update();










    }
    @Override
    public void stop(){
        try {
            myWriter = new FileWriter("TeleopMovementData.txt");
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

}
