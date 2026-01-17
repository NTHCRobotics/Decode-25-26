package org.firstinspires.ftc.teamcode;
//IMPORTS
import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp(name = "mainDrive", group = "Axolotl")
public class mainDrive extends OpMode{
    //Hardware
    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR, intakeB, flyWheelA, flyWheelB; //Motors
    private Servo magazineServo, loadServo; //Servos
    private NormalizedColorSensor colorSensorL; //Left Color Sensor
    private NormalizedColorSensor colorSensorBR; //Right Back Color Sensor
//    private NormalizedColorSensor colorSensorFR; //Right Front Color Sensor

    //April Tag + Vision Portal stuff
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;



    //Variables
    double speedMod = 0.5; //Speed of wheel motors (around 1/2 maximum rate)
    double aprilTagReadAttempts = 1;
    double[] magazineReadPositions = {0.0, 360.0/3/300, 360.0/3/300*2}; //lists out 1/3rd rotations of the magazine. The weird math is to normalize it to [0,1] given the servo's 300 degree range.
    // !IMPORTANT! artifactOrder is an ArrayList since it needs to be repeatedly scanned and edited.
    // 0 --> L, 1 --> BR, 2 --> FR
    ArrayList<String> magazineOrder = new ArrayList<String>(); //correlates to the artifact color currently in readPositions (e.g. the artifacts in the magazine)

    String[] obeliskOrder = {null, null, null}; //Correlates to the competition artifact sequence (read at init(), never changed)


    @Override
    public void init(){
        //Hardware
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        flyWheelA = hardwareMap.get(DcMotorEx.class, "flyWheelA");
        flyWheelB = hardwareMap.get(DcMotorEx.class, "flyWheelB");
        //Servos
        magazineServo = hardwareMap.get(Servo.class, "magazineServo");
        loadServo = hardwareMap.get(Servo.class, "loadServo");
        //Sensors
        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");
        colorSensorBR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorBR");
//        colorSensorFR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFR");

        //Initialize magazine orders will null (so size = 3)
        magazineOrder.add("empty");
        magazineOrder.add("empty");
        magazineOrder.add("empty");

        //Initialize AprilTag Detection
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //Settings here
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();


        //Initialization
        magazineServo.setPosition(0.0);
        loadServo.setPosition(0.0);

        //AprilTag detection
        try {
            obeliskOrder = detectObelisk();
        } catch (InterruptedException e) {
            obeliskOrder = new String[]{"green", "purple", "purple"}; // DEFAULT ORDER FOR ISSUES
        }

    }

    @Override
    public void init_loop(){
        try {
            obeliskOrder = detectObelisk();
            sleep(100); //Sleep time so it probably won't crash with unnecessary reads. It reads throughout init regardless.
        } catch (InterruptedException e) {
            obeliskOrder = new String[]{"green", "purple", "purple"}; // DEFAULT ORDER FOR ISSUES RATHER THAN THROWING A RANDOM ERROR. IT'S RIGHT 33.33% OF THE TIME AT LEAST
        }
        updateTelemetry();
    }

    @Override
    public void loop(){
        drive(); // translation and rotation
        //spinIntakes(); //Spinning the intakes (duh) DISABLED UNTIL BUTTON IS BOUND
        indexArtifacts(); // Keeps a running list of what artifacts exist within the magazine
        launchArtifactManual();
        updateTelemetry(); // updates driver's hub telemetry
    }
    //Custom Classes
    //Read obelisk
    public void readObelisk() {
        String[] obeliskOrder = {"green", "purple", "green"}; //Temporary return while apriltag read is in progress
    }

    //Basic movement
    public void drive() {
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;

        //Wheel-by-wheel spin calculations
        double FL = (-x - y - rotation) * speedMod;
        double FR = (-x + y - rotation) * speedMod;
        double BL = (x - y - rotation) * speedMod;
        double BR = (x + y - rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }

    public void launchArtifactManual(){
        // change magazine order = the dpad brah
        // move elevator up = right bumper
        // move fly wheels = right trigger ya
        // setposition double is degree so like 1 is 180 and 0.5 is 90
        // degrees vary cause idk them

        if (gamepad2.dpad_left){
            magazineServo.setPosition(0.6);
        } else if (gamepad2.dpad_right){
            magazineServo.setPosition(0.4);
        } else if (gamepad2.dpad_up){
            magazineServo.setPosition(0.2);
        }

        if (gamepad2.right_bumper){
            loadServo.setPosition(1);
        } else {
            loadServo.setPosition(0.02);
        }

        double triggerValue = gamepad2.right_trigger;

        if (triggerValue > 0.1) {
            flyWheelA.setPower(1);
            flyWheelB.setPower(1);
        } else {
            flyWheelA.setPower(0);
            flyWheelB.setPower(0);
        }
    }

    //Intake control
    public void spinIntakes() {
        intakeB.setPower(0.5);
    }

    //Color sensor
    public String colorMatch(NormalizedColorSensor colorSensor){
        double red = colorSensor.getNormalizedColors().red;
        double green = colorSensor.getNormalizedColors().green;
        double blue = colorSensor.getNormalizedColors().blue;

        double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
        double greenMatch = Math.max(0, green - Math.max(red, blue));

        double purpleTolerance = 0.5;
        double greenTolerance = 0.5;

        if (purpleMatch < purpleTolerance && greenMatch < greenTolerance){
            return "empty";
        }
        else if (purpleMatch > greenMatch){
            return "purple";
        }
        else {
            return "green";
        }
    }

    public void indexArtifacts() {
        String colorL = colorMatch(colorSensorL);
        String colorBR = colorMatch(colorSensorBR);
//        String colorFR = colorMatch(colorSensorFR);

        magazineOrder.set(0, colorL);
        magazineOrder.set(1, colorBR);
//        magazineOrder.set(2, colorFR);
    }

    public void launchInOrder() throws InterruptedException {
        for (int i = 0; i < 3; i++){
            int index = magazineOrder.indexOf(obeliskOrder[i]);
            magazineServo.setPosition(magazineReadPositions[index]);
            sleep(200); //Allows for the magazine to rotate
            //LAUNCH CODE HERE
            magazineOrder.set(i, "empty");
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Magazine Order", magazineOrder.get(0) + " " + magazineOrder.get(1) + " " + magazineOrder.get(2));
        telemetry.addData("AprilTag Read Attempts", aprilTagReadAttempts);
        telemetry.addData("Obelisk Order", obeliskOrder[0] + " " + obeliskOrder[1] + " " + obeliskOrder[2]);
    }

    public String[] detectObelisk() throws InterruptedException {
        ArrayList<AprilTagDetection> obeliskDetections = aprilTagProcessor.getDetections(); //Gets the first detection of an apriltag, should only be the center one
        if (obeliskDetections.isEmpty()) {
            return null;
        } else {
            int id = obeliskDetections.get(0).id; //Gets the first element, only ONE element should be in frame.
            switch (id) {
                case 21:
                    return new String[]{"green", "purple", "purple"};
                case 22:
                    return new String[]{"purple", "green", "purple"};
                case 23:
                    return new String[]{"purple", "purple", "green"};
                default:
                    throw new InterruptedException(); //REFERS TO DEFAULT FAILURE HANDLING IN CATCH/TRY
            }
        }

    }

}
