package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="classTestProgramTeleop")
public class classTestProgramTeleop extends OpMode{
    int x;

    @Override
    public void init() {
        x = 5;
    }

    public void loop() {
        x = x+5;

    }
}
