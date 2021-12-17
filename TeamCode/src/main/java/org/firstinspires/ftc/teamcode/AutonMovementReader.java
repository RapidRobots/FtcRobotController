/*
 * Blue program for 1 side
 *
 *
 * */





package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name="AutonMovementReader")
public class AutonMovementReader extends LinearOpMode {




    boolean switchedRead = false;

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    boolean leftRead = true;

    boolean rightRead = false;

    String leftPowerStore;
    String rightPowerStore;

    char storedChar;

    String readChar;

    String topLeftDiagonalPower;

    String topRightDiagonalPower;

    String fileData;

    FileWriter myWriter =  new FileWriter(String.format("%s/FIRST/data/TeleopMovementData.txt", Environment.getExternalStorageDirectory().getAbsolutePath()));

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

    public AutonMovementReader() throws IOException {
    }

    public void print(String data) {
        telemetry.addData(data,"" );

    }
    public void printSend(){
        telemetry.update();
    }


    @Override
    public void runOpMode() {


        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_rear");
        rightBackMotor = hardwareMap.dcMotor.get("right_rear");

        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        armRotateMotor = hardwareMap.dcMotor.get("arm_rotate");
        armExtendMotor = hardwareMap.dcMotor.get("arm_extend");

        intake_motor = hardwareMap.dcMotor.get("intake_motor");


        waitForStart();


        String filename = "TeleopMovementData";


        File file = AppUtil.getInstance().getSettingsFile(filename);

        fileData = ReadWriteFile.readFile(file);

        telemetry.addData("Motor Values", fileData);

/*for (int i = 0; i < (fileData.length()); i++) {
            storedChar = fileData.charAt(i);
            readChar = String.valueOf(storedChar);
            if (readChar != "-" || readChar != "/" && leftRead == true) {
                leftPowerStore = leftPowerStore + readChar;
                readChar = null;
            }
            if (readChar != "-" || readChar != "/" && rightRead == true) {
                rightPowerStore = leftPowerStore + readChar;
                readChar = null;
            }
            if (readChar == "-" && switchedRead == false && leftRead == true) {
                switchedRead = true;
                leftRead = false;
                rightRead = true;
            }
            if (readChar == "-" && switchedRead == false && leftRead == false) {
                switchedRead = true;
                leftRead = true;
                rightRead = false;
            }
            if (readChar == "/") {
                topLeftDiagonal = Double.parseDouble(leftPowerStore);
                topRightDiagonal = Double.parseDouble(rightPowerStore);
                leftFrontMotor.setPower(topLeftDiagonal);//
                rightBackMotor.setPower(-topLeftDiagonal);//

                rightFrontMotor.setPower(-topRightDiagonal);
                leftBackMotor.setPower(topRightDiagonal);
                leftPowerStore = null;
                rightPowerStore = null;
            }
            if (readChar == "_") {
                sleep(10);
            }
        }*/ //Important comment. Old code before fix and move


        for (int i = 0; i < fileData.length(); i++) {
            if (readChar != "-" && readChar != "/" && leftRead == true) {
                leftPowerStore = leftPowerStore + readChar;
                readChar = "";
            }

            if (readChar != "-" && readChar != "/" && rightRead == true) {
                rightPowerStore = leftPowerStore + readChar;
                readChar = "";
            }

            if (readChar == "-" && switchedRead == false && leftRead == true) {

                leftRead = false;
                rightRead = true;

                switchedRead = true;
            }

            if (readChar == "-" && switchedRead == false && leftRead == false) {
                switchedRead = true;
                leftRead = true;
                rightRead = false;
            }

            if (readChar == "/") {
                print(leftPowerStore);
                print(rightPowerStore);
                printSend();

                leftPowerStore = "";
                rightPowerStore = "";
            }
            if (readChar == "_") {
                sleep(10);

                switchedRead = false;
            }

            stop();

        }
    }




}
