/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpPart2", group="Iterative Opmode")

public class TeleOpPart2 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    // private Servo leftServo = null;
    // private Servo rightServo = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotor.class, "motorTopLeft");
        frontRight = hardwareMap.get(DcMotor.class, "motorTopRight");
        backLeft = hardwareMap.get(DcMotor.class, "motorBottomLeft");
        backRight = hardwareMap.get(DcMotor.class, "motorBottomRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // leftServo.setDirection(Servo.Direction.FORWARD);
        // rightServo.setDirection(Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        /*double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;*/

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
         leftPower  = -gamepad1.left_stick_y ;
         rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        /*if (gamepad1.left_stick_y > 0){
            frontLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(gamepad1.left_stick_y);
        }else if (gamepad1.left_stick_y < 0){
            frontLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(gamepad1.left_stick_y);
        }*/
        
        /*frontLeft.setPower(-leftPower);
        frontRight.setPower(-rightPower);
        backLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);
        
        if (gamepad1.left_trigger > 0){
            frontLeft.setPower(gamepad1.left_trigger);
            frontRight.setPower(-gamepad1.left_trigger);
            backLeft.setPower(-gamepad1.left_trigger);
            backRight.setPower(gamepad1.left_trigger);
        }else if (gamepad1.right_trigger > 0){
            frontLeft.setPower(-gamepad1.right_trigger);
            frontRight.setPower(gamepad1.right_trigger);
            backLeft.setPower(gamepad1.right_trigger);
            backRight.setPower(-gamepad1.right_trigger);
        }*/
        
        //Strafing
        
        if (gamepad1.left_trigger > 0){
            frontLeft.setPower(gamepad1.left_trigger);
            frontRight.setPower(-gamepad1.left_trigger);
            backLeft.setPower(-gamepad1.left_trigger);
            backRight.setPower(gamepad1.left_trigger);
        }else if (gamepad1.right_trigger > 0){
            frontLeft.setPower(-gamepad1.right_trigger);
            frontRight.setPower(gamepad1.right_trigger);
            backLeft.setPower(gamepad1.right_trigger);
            backRight.setPower(-gamepad1.right_trigger);
        }
        
        //Drive
        
        frontLeft.setPower(0.85*gamepad1.left_stick_y);
        frontRight.setPower(0.85*gamepad1.left_stick_y);
        backLeft.setPower(0.85*gamepad1.left_stick_y);
        backRight.setPower(0.85*gamepad1.left_stick_y);
        
        //Boosted Drive
        if(gamepad1.b){
         frontLeft.setPower(-0.85);
        frontRight.setPower(-0.85);
        backLeft.setPower(-0.85);
        backRight.setPower(-0.85);
        }
        
        
        //Auto Red
        runtime.reset();
        while (gamepad1.x && (runtime.seconds() < 2.0)) {
            frontLeft.setPower(0.50);
            frontRight.setPower(-0.50);
            backLeft.setPower(-0.50);
            backRight.setPower(0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Auto Blue
        runtime.reset();
        while (gamepad1.dpad_up && (runtime.seconds() < 1.5)) {
            frontLeft.setPower(-0.50);
            frontRight.setPower(0.50);
            backLeft.setPower(0.50);
            backRight.setPower(-0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        
        
    //Turn
        
        if (gamepad1.right_stick_x > 0){
            frontLeft.setPower(-0.55*gamepad1.right_stick_x);
            frontRight.setPower(0.55*gamepad1.right_stick_x);
            backLeft.setPower(-0.55*gamepad1.right_stick_x);
            backRight.setPower(0.55*gamepad1.right_stick_x);
        }else if (gamepad1.right_stick_x < 0){
            frontLeft.setPower(-0.55*gamepad1.right_stick_x);
            frontRight.setPower(0.55*gamepad1.right_stick_x);
            backLeft.setPower(-0.55*gamepad1.right_stick_x);
            backRight.setPower(0.55*gamepad1.right_stick_x);
        }
        
    //Quick Turn
        if(gamepad1.y){
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
        //Arm
        
        //Variable for Arm Control
        
        if (gamepad2.left_stick_y > 0){
            arm.setPower(0.3);
        }else if(gamepad2.left_stick_y < 0){
            arm.setPower(-0.7);
        } else{
            arm.setPower(0);
        
        }
        if (gamepad2.right_stick_y > 0){
            arm2.setPower(0.6);
        }else if(gamepad2.right_stick_y < 0){
            arm2.setPower(-0.8);
        } else{
            arm2.setPower(0);
        }
        //Servo
        
        // if (gamepad2.left_trigger > 0){
        //     leftServo.setPosition(.5);
        //     rightServo.setPosition(0.2);
        // }else if (gamepad2.right_trigger > 0){
        //     leftServo.setPosition(0.28);
        //     rightServo.setPosition(.48);
        // }
        
        

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
