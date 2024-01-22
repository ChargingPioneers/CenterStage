/* Copyright (c) 2017 FIRST. All rights reserved.


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.PIXEL_FORMAT;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.vuforia.Image;
import com.vuforia.Vuforia;
import java.nio.ByteBuffer;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 
@Disabled
@Autonomous(name="Autonomous20204", group="Pushbot")

public class Autonomous20204 extends LinearOpMode {

    /* Declare OpMode members. 
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor arm = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    
    //Shawn
    private Blinker expansion_Hub_4;
    DigitalChannel touch;
    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    private static final String VUFORIA_KEY = "AVPRW+T/////AAABmYg0Njwhc0n/teI+7Sz8f/Baxyp0o6W48fBEflz8RZs3G/bVjI/5PyebGV6SkXhE1unHTRVzOVCo2cuuePhML8YCeHWm1dHZ2KbshLfc/yne7rfe2VaKPR3rrJXPF5CdMTWj4nTxm6w7KxiqvtvF2p2si1FrculcXUwbHeZ9X3O6VSntXMuNJDxXJEC3O5hT5kb7ZzsSWlot9YfUqJRxttrYYz8Xu1D2IhtOs26a2A9FC8afgGouyHucBDfl+WP59+H6wYaXRbyvcFdytq9Fp7mlSsA9RA6DV70PtJWDmehLO5hhOKq4ihVNCjJcgG38UefDAyDhWWMdjRwsiaVctq6QkmG1oMuTfIF1Dun2lDpZ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private final int numberOfRowsToScanInImage = 30;
    private final double BATTERY_LEVEL = 1;
    private final double DrivePower = 0.85;
    private ParkingSpace parkingSpace = ParkingSpace.UNO;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    
    
    
    enum ParkingSpace {
        UNO,
        DOS,
        TRES
    }
    
    
    

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //boolean isCameraReady = getCameraReady();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        getCameraReady();
        waitForStart();
        parkingSpace = getCameraReading();
        switch(parkingSpace){
        case UNO:
               runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8.0)) {
            frontLeft.setPower(-0.10);
            frontRight.setPower(-0.10);
            backLeft.setPower(-0.10);
            backRight.setPower(-0.10);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        return;
        case DOS:
               runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(-0.45);
            frontRight.setPower(-0.45);
            backLeft.setPower(-0.45);
            backRight.setPower(-0.45);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
            telemetry.addData("dos", "dos");
            telemetry.update();
        return;
        case TRES:
               runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(-0.60);
            frontRight.setPower(0.60);
            backLeft.setPower(0.60);
            backRight.setPower(-0.60);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() >= 1.0) && (runtime.seconds() < 2.0)) {
            frontLeft.setPower(-0.40);
            frontRight.setPower(-0.40);
            backLeft.setPower(-0.40);
            backRight.setPower(-0.40);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
            telemetry.addData("tres", "tres");
            telemetry.update();
        return;
        }
        /*telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(50);
        //The actual program
        eTime.reset();

        if (isCameraReady) {
            parkingSpace = getCameraReading();
        }*/

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backward for 1 Second
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
        
        
        //Simple Auto for Red side Red Terminal
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(0.50);
            frontRight.setPower(-0.50);
            backLeft.setPower(-0.50);
            backRight.setPower(0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(-0.70);
            frontRight.setPower(-0.70);
            backLeft.setPower(-0.70);
            backRight.setPower(-0.70);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() >= 1.0) && (runtime.seconds() < 2.0)) {
            frontLeft.setPower(-0.04);
            frontRight.setPower(0.04);
            backLeft.setPower(-0.04);
            backRight.setPower(0.04);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() >= 2.0) && (runtime.seconds() < 3.0)) {
            arm.setPower(-0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0);
        while (opModeIsActive() && (runtime.seconds() >= 3.0) && (runtime.seconds() < 4.0)) {
            arm.setPower(0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0);
        */
        //Simple Auto for Red side Blue Terminal
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(0.50);
            frontRight.setPower(-0.50);
            backLeft.setPower(-0.50);
            backRight.setPower(0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
        
        //Simple Auto for Blue side Blue Terminal
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(-0.50);
            frontRight.setPower(0.50);
            backLeft.setPower(0.50);
            backRight.setPower(-0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
        
        //Simple Auto for Blue side Red Terminal
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            frontLeft.setPower(-0.50);
            frontRight.setPower(0.50);
            backLeft.setPower(0.50);
            backRight.setPower(-0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
        
        
        //It isn't working correctly
        
        //Arm step 1
        /*runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() >= 1.0) && (runtime.seconds() < 2.0)) {
            frontLeft.setPower(0.30);
            frontRight.setPower(0.30);
            backLeft.setPower(0.30);
            backRight.setPower(0.30);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 2
        while (opModeIsActive() && (runtime.seconds() >= 2.0) && (runtime.seconds() < 3.0)) {
            frontLeft.setPower(-0.23);
            frontRight.setPower(0.23);
            backLeft.setPower(-0.23);
            backRight.setPower(0.23);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 3
        while (opModeIsActive() && (runtime.seconds() >= 3.0) && (runtime.seconds() < 4.0)) {
            arm.setPower(0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0);
        //Arm step 4
        while (opModeIsActive() && (runtime.seconds() >= 3.0) && (runtime.seconds() < 4.0)) {
            frontLeft.setPower(0.15);
            frontRight.setPower(0.15);
            backLeft.setPower(0.15);
            backRight.setPower(0.15);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 5
        while (opModeIsActive() && (runtime.seconds() >= 4.0) && (runtime.seconds() < 5.0)) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 6
        while (opModeIsActive() && (runtime.seconds() >= 5.0) && (runtime.seconds() < 6.0)) {
            frontLeft.setPower(-0.15);
            frontRight.setPower(-0.15);
            backLeft.setPower(-0.15);
            backRight.setPower(-0.15);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 7
        while (opModeIsActive() && (runtime.seconds() >= 6.0) && (runtime.seconds() < 7.0)) {
            arm.setPower(-0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0);
        //Arm step 8
        while (opModeIsActive() && (runtime.seconds() >= 7.0) && (runtime.seconds() < 8.0)) {
            frontLeft.setPower(0.23);
            frontRight.setPower(-0.23);
            backLeft.setPower(0.23);
            backRight.setPower(-0.23);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 9
        while (opModeIsActive() && (runtime.seconds() >= 8.0) && (runtime.seconds() < 9.0)) {
            frontLeft.setPower(-0.30);
            frontRight.setPower(-0.30);
            backLeft.setPower(-0.30);
            backRight.setPower(-0.30);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm step 10
        while (opModeIsActive() && (runtime.seconds() >= 9.0) && (runtime.seconds() < 10.0)) {
            frontLeft.setPower(-0.50);
            frontRight.setPower(0.50);
            backLeft.setPower(0.50);
            backRight.setPower(-0.50);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
        
        /*while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() >= 2.0) && (runtime.seconds() < 4.0)) {
            arm.setPower(-0.5);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0);
        while (opModeIsActive() && (runtime.seconds() >= 4.0) && (runtime.seconds() < 6.0)) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        
    }        
        //SHAWN'S CODE
        
        
        
        
        //Copy from here!!!!

//Land Function

     //Initialize the Vuforia localization engine.
    private void initVuforia(){
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        
         //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Tony");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(10);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    
     //Initialize the TensorFlow ObjThe arguments TFOD_MODEL_ASSET, LABELS are defined earlier in the op mode and are season specific.ect Detection engine.
     
    private void initTfod(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    }

    private boolean getCameraReady(){
        initVuforia();
        initTfod();

        
         //Activate TensorFlow Object Detection before we wait for the start command.
         //Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         
        if (tfod != null) {
            tfod.activate();
            telemetry.addData("TFOD is Activated", "");

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        if (tfod == null) {
            telemetry.addData("TFOD is Null", "");
            telemetry.update();
            return false;
        }
        return true;
    }

    private ParkingSpace getCameraReading() {
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch(Exception e) {
            telemetry.addData("e", "E");
            telemetry.update();
        }
        if (frame == null) return ParkingSpace.UNO;
        long numImages = frame.getNumImages();
        Image image = null;
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(i);
            }
        }

        int[] colors = {0, 0, 0, 0};

        if (image != null) {
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
            int imgWidth = image.getWidth();
            int imgHeight = image.getHeight();
            int[] startingIndexes = getRowStartingIndexes(imgHeight, imgWidth, numberOfRowsToScanInImage);
            for (int i = numberOfRowsToScanInImage / 3; i < numberOfRowsToScanInImage * 2 / 3; i++) {
                for (int j = startingIndexes[i] + imgWidth * 2 / 3; j < startingIndexes[i] + imgWidth * 2 * 2/3; j += 2) {
                    colors[getColor(pixelArray[j], pixelArray[j+1])]++;
                    telemetry.addData("width", imgWidth);
                    telemetry.addData("height", imgHeight);
                    telemetry.addData("startingIndexes[i]", startingIndexes[i]);
                    telemetry.addData("yellow", colors[0]);
                    telemetry.addData("green", colors[1]);
                    telemetry.addData("purple", colors[2]);
                    telemetry.update();
                    // sleep(30000);
                }
            }
            // sleep(5000);
            // telemetry.addData("b1", pixelArray[0]);
            // telemetry.addData("b2", pixelArray[1]);
            telemetry.addData("yellow", colors[0]);
            telemetry.addData("green", colors[1]);
            telemetry.addData("purple", colors[2]);
            telemetry.update();
            for (int i = 0; i < startingIndexes.length; i++)
                telemetry.addData("startingIndexes[i]", startingIndexes[i]);
            telemetry.update();
            // sleep(30000);
        }

        frame.close();
        int max_index = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[max_index])
                max_index = i;
        }

        if (max_index == 0)
            return ParkingSpace.UNO;
        if (max_index == 1)
            return ParkingSpace.DOS;
        return ParkingSpace.TRES;
    }

    private int[] getRowStartingIndexes(int height, int width, int numRows) {
        int[] newArr = new int[numRows];
        int stepSize = 2 * height / numRows * width;
        for (int i = 1; i < numRows; i++) {
            newArr[i] = i * stepSize;
        }
        return newArr;
    }

    private int getColor(byte b1, byte b2) {
        // GGGBBBBB RRRRRGGG;
        String s1 = String.format("%8s", Integer.toBinaryString(b2 & 0xFF)).replace(' ', '0');
        String s2 = String.format("%8s", Integer.toBinaryString(b1 & 0xFF)).replace(' ', '0');
        // RRRRRGGG GGGBBBBB;
        int[] color = new int[3];
        String r = s1.substring(0, 5);
        String g = s1.substring(5) + s2.substring(0, 3);
        String b = s2.substring(3);
        color[0] = convertBitStringToInt(r);
        color[1] = convertBitStringToInt(g);
        color[2] = convertBitStringToInt(b);
        double[] hsv = convertRGBtoHSV(color);
        telemetry.addData("hsv", hsv[0]);
        telemetry.addData("hsv", hsv[1]);
        telemetry.addData("hsv", hsv[2]);
        telemetry.addData("b1", b1);
        telemetry.addData("b2", b2);
        telemetry.addData("hsv[2]", hsv[2]);
        if (hsv[0] >= 34 && hsv[0] <= 67 && hsv[1] > 0.41 && hsv[2] > 0.36)
            return 0;
        if (hsv[0] >= 81 && hsv[0] <= 143 && hsv[1] > .28 && hsv[2] > 0.12)
            return 1;
        if (hsv[0] >= 258 && hsv[0] <= 320 && hsv[1] > 0.33 && hsv[2] > 0.21)
            return 2;
        return 3;
    }

    private double[] convertRGBtoHSV(int[] rgb) {
        double rPrime = (double) rgb[0]/31;
        double gPrime = (double) rgb[1]/63;
        double bPrime = (double) rgb[2]/31;
        double cMax = Math.max(rPrime, Math.max(gPrime, bPrime));
        double cMin = Math.min(rPrime, Math.min(gPrime, bPrime));
        double delta = cMax - cMin;
        double[] hsv = new double[3];

        // calculate hue
        if (delta == 0)
            hsv[0] = 0;
        else if (cMax == rPrime) {
            double temp = ((gPrime - bPrime) / delta) % 6;
            if (temp < 0)
                temp += 6;
            hsv[0] = 60 * temp;
        }
        else if (cMax == gPrime)
            hsv[0] = 60 * (((bPrime - rPrime) / delta) + 2);
        else
            hsv[0] = 60 * (((rPrime - gPrime) / delta) + 4);

        // calculate saturation
        if (cMax == 0)
            hsv[1] = 0;
        else
            hsv[1] = delta / cMax;

        // calculate value
        hsv[2] = cMax;

        return hsv;
    }

    private int convertBitStringToInt(String s) {
        int sum = 0;
        // Little Endian
        // int digit = 0;
        // for (char c : s.toCharArray()) {
        //     if (c == '1') {
        //         sum += Math.pow(2, digit);
        //     }
        //     digit++;
        // }
        // Big Endian
        int digit = s.length() - 1;
        for (char c : s.toCharArray()) {
            if (c == '1') {
                sum += Math.pow(2, digit);
            }
            digit--;
        }
        return sum;
    }
}*/
