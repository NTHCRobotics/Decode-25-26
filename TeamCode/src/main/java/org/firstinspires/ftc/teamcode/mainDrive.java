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
    private NormalizedColorSensor colorSensorFR; //Right Front Color Sensor

    //April Tag + Vision Portal stuff
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;



    //Variables
    double speedMod = 0.5; //Speed of wheel motors (around 1/2 maximum rate)
    double aprilTagReadAttempts = 1;
    double[] magazineReadPositions = {0.0, 360.0/3/300, 360.0/3/300*2}; //lists out 1/3rd rotations of the magazine. The weird math is to normalize it to [0,1] given the servo's 300 degree range.
    double[] loadServoPositions = {0.0, 90.0/300}; // Omar said 90 degrees sooooooooooo

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
        colorSensorFR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorFR");

        //Initialize magazine orders with null (so size = 3)
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
        if (!gamepad2.dpad_down) { // Safe initialization mode for redundancy, holding gamepad2 dpad down while passing from init_loop() to loop() will prevent the AprilTag processor from running.
            try { //TRY/CATCH BECAUSE IT'S REQUIRED WHEN USING SLEEP()
                obeliskOrder = detectObelisk() == null ? detectObelisk() : obeliskOrder; //Kinda weird, will replace later, if detectObelisk() returns null, it ignores it and keeps whatever the last value was.
            } catch (InterruptedException e) {
                obeliskOrder = new String[]{"green", "purple", "purple"}; // DEFAULT ORDER FOR ISSUES RATHER THAN THROWING A RANDOM ERROR. IT'S RIGHT 33.33% OF THE TIME AT LEAST
            }
        }
        updateInitTelemetry();
    }

    @Override
    public void loop(){
        drive(); // translation and rotation
        //spinIntakes(); //Spinning the intakes (duh) DISABLED UNTIL BUTTON IS BOUND
        indexArtifacts(); // Keeps a running list of what artifacts exist within the magazine
        launchArtifactManual(); //Manual artifact launching, default for now unless we cna get something crazy working.
        updateTelemetry();
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
        // setposition double is degree so like 1 is 180 and 0.5 is 90 <-- This is not true, you're probably thinking of radians since 180 is (pi) and 90 is (pi)/2. Unfortunately, our servos are [0, 1] between [0, 300] degrees
        // degrees vary cause idk them

        //Fixed your degrees, Danya. Also, left is 0, up is 1, and right is 2.

        if (gamepad2.dpad_left){
            magazineServo.setPosition(magazineReadPositions[0]);
        } else if (gamepad2.dpad_up){
            magazineServo.setPosition(magazineReadPositions[1]);
        } else if (gamepad2.dpad_right){
            magazineServo.setPosition(magazineReadPositions[2]);
        }

        //Idk if these are the right values for the launcher but I'm going to assume they are.
        if (gamepad2.right_bumper){
            loadServo.setPosition(loadServoPositions[1]);
        } else {
            loadServo.setPosition(loadServoPositions[0]);
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
    public String colorMatch(NormalizedColorSensor colorSensor, double purpleTolerance, double greenTolerance, int steps){
        double purpleSum = 0;
        double greenSum = 0;

        for (int i = 0; i < steps; i++){ // Takes a rolling account of readings with a certain number of steps, time unbounded, to prevent one-off errors. Unlike AprilTags where the error is often blurred motion which requires a short wait time to fix, color sensors are just kinda jank so sampling right after is usually fine.
            double red = colorSensor.getNormalizedColors().red;
            double green = colorSensor.getNormalizedColors().green;
            double blue = colorSensor.getNormalizedColors().blue;

            double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
            double greenMatch = Math.max(0, green - Math.max(red, blue));

            purpleSum += purpleMatch;
            greenSum += greenMatch;
        }

        double purpleAvg = purpleSum / steps; // Determines the average percentage [0, 1] that whatever is scanned matches purple
        double greenAvg = greenSum / steps; // Determines the average percentage [0, 1] that whatever is scanned matches green

        if (purpleAvg < purpleTolerance && greenAvg < greenTolerance){
            return "empty";
        }
        else if (purpleAvg > greenAvg){
            return "purple";
        }
        else {
            return "green";
        }
    }

    public void indexArtifacts() {
        String colorL = colorMatch(colorSensorL, 0.5, 0.5, 100);
        String colorBR = colorMatch(colorSensorBR, 0.5, 0.5, 100);
        String colorFR = colorMatch(colorSensorFR, 0.5, 0.5, 100);

        magazineOrder.set(0, colorL);
        magazineOrder.set(1, colorBR);
        magazineOrder.set(2, colorFR);
    }

    public void launchInOrder() throws InterruptedException {
        for (int i = 0; i < 3; i++){
            //Explanation: for each ball in the magazine, match its position with an index in the obelisk order, set position to the [i]th ball, load it, then retract and move onto the next.
            int index = magazineOrder.indexOf(obeliskOrder[i]);
            if (index == -1){
                continue; //Skip to the next part if it can't find the ball
            }
            magazineServo.setPosition(magazineReadPositions[index]);
            sleep(200); //Allows for the magazine to rotate
            loadServo.setPosition(loadServoPositions[1]);
            sleep(1000);
            loadServo.setPosition(loadServoPositions[0]);
            sleep(100);
            magazineOrder.set(i, "empty");
        }
    }

    public void updateInitTelemetry() {
        telemetry.addData("Magazine Order", magazineOrder.get(0) + " " + magazineOrder.get(1) + " " + magazineOrder.get(2));
        telemetry.addData("AprilTag Read Attempts", aprilTagReadAttempts);
        telemetry.addData("Obelisk Order", obeliskOrder[0] + " " + obeliskOrder[1] + " " + obeliskOrder[2]);
        telemetry.update();
    }

    public void updateTelemetry() {
        telemetry.addData("Magazine Order", magazineOrder.get(0) + " " + magazineOrder.get(1) + " " + magazineOrder.get(2));
        telemetry.addData("Wheel Powers", wheelFL.getPower() + "    " + wheelFR.getPower() + "\n"
                                                      +wheelBL.getPower() + "     " + wheelBR.getPower()); //Hopefully this serves as a bit of a better display, remove if not.
    }

    // Scans the obelisk, returns either null for "can't see anything" and "ID out of range" or a 3-value String[] if it identifies an appropriate AprilTag code
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
                    return null; //REFERS TO DEFAULT FAILURE HANDLING IN LOOP
            }
        }

    }

}
