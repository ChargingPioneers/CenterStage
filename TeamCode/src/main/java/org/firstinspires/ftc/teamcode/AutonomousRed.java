package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.tfod.FrameConsumer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red Short Auto")

public class AutonomousRed extends LinearOpMode {
    
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    @Override
    
    public void runOpMode()
    {
        

    fL = hardwareMap.dcMotor.get ("fL");
    fR = hardwareMap.dcMotor.get("fR");
    bL = hardwareMap.dcMotor.get ("bL");
    bR = hardwareMap.dcMotor.get("bR");
    
    fL.setDirection(DcMotor.Direction.REVERSE);
    bL.setDirection(DcMotor.Direction.REVERSE);
    
    fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    
    waitForStart();
    
    fL.setPower(-0.5);
    fR.setPower(0.5);
    bL.setPower(0.5);
    bR.setPower(-0.5);
    sleep(1700);
    
    }

}

