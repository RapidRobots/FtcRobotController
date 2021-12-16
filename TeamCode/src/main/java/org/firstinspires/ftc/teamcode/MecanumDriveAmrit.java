package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name="MecanumDriveAmrit")
public class MecanumDriveAmrit extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    double topLeftDiagonal= 0.0;//left front and right back
    double topRightDiagonal= 0.0;//right front and left back
    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_rear");
        rightBackMotor = hardwareMap.dcMotor.get("right_rear");

        waitForStart();

        while (opModeIsActive()){
            double forward = gamepad1.left_stick_y; // 0
            double strafe = gamepad1.left_stick_x; // 1

            topLeftDiagonal = forward+strafe; // + =
            topRightDiagonal = forward - strafe;//- =

            leftFrontMotor.setPower(topLeftDiagonal);//
            rightBackMotor.setPower(topLeftDiagonal);//

            rightFrontMotor.setPower(topRightDiagonal);
            leftBackMotor.setPower(topRightDiagonal);
        }
    }
}

