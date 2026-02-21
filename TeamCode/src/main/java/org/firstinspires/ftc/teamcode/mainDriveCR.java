package org.firstinspires.ftc.teamcode;
//IMPORTS
import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "mainDriveCR", group = "Axolotl")
public class mainDriveCR extends OpMode{
    //Hardware
    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR, intakeB, intakeA, flyWheelA, flyWheelB; //Motors
    private Servo magazineServo, loadServo;
    private CRServo hoodServo; //Servos
    private ColorSensor colorSensorL, colorSensorBR, colorSensorFR; //Color sensors
    private DistanceSensor colorDistanceL, colorDistanceBR, colorDistanceFR;                                                               

    //April Tag + Vision Portal stuff
    //AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;


    //Variables
    boolean intake_doneOnce = false;
    double previousMagazine = 0; //I'm sure there's a better way to do this
    double speedMod = 1;
    double aprilTagReadAttempts = 1;
    double offset = 0;
    double[] magazineShootingPositions = {0.0539, 0.4968, 0.9366}; // Omar said 90 degrees sooooooooooo
    double[] magazineIntakePositions = {0.1665, 0.3826}; // [0] is Back, [1] is front
    String[] magazineIndex = new String[]{null, null, null};
    int magazineCurrentIndex = 0;
    //double[] loadServoPositions = {0.6, /*90 / 300*/ 0.8 , (90 / 300) - 0.1}; // Omar said 90 degrees sooooooooooo
    double[] loadServoPositions = {0.0, 0.4, 0.9366}; // Omar said 90 degrees sooooooooooo
    boolean movingElevator = false;

    boolean dPadRightDown = false; //TEST
                                   //
    String[] obeliskOrder = new String[]{"green", "purple", "purple"}; // DEFAULT ORDER FOR ISSUES RATHER THAN THROWING A RANDOM ERROR. IT'S RIGHT 33.33% OF THE TIME AT LEAST
    //String[] obeliskOrder = {null, null, null} ; //Correlates to the competition artifact sequence (read at init(), never changed)

    // !IMPORTANT! artifactOrder is an ArrayList since it needs to be repeatedly scanned and edited.
    // 0 --> L, 1 --> BR, 2 --> FR
    ArrayList<String> magazineOrder = new ArrayList<String>(); //correlates to the artifact color currently in readPositions (e.g. the artifacts in the magazine)



    @Override
    public void init(){
        //Hardware
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");


        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");

        flyWheelA = hardwareMap.get(DcMotorEx.class, "flyWheelA");
        flyWheelB = hardwareMap.get(DcMotorEx.class, "flyWheelB");
        //Servos
        magazineServo = hardwareMap.get(Servo.class, "magazineServo");
        loadServo = hardwareMap.get(Servo.class, "loadServo");
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        //Sensors
        colorSensorL  = hardwareMap.get(ColorSensor.class, "colorSensorL");
        colorSensorBR = hardwareMap.get(ColorSensor.class, "colorSensorBR");
        colorSensorFR = hardwareMap.get(ColorSensor.class, "colorSensorFR");

        colorDistanceL  = hardwareMap.get(DistanceSensor.class, "colorSensorL");
        colorDistanceBR = hardwareMap.get(DistanceSensor.class, "colorSensorBR");
        colorDistanceFR = hardwareMap.get(DistanceSensor.class, "colorSensorFR");

        //Initialize magazine orders with null (so size = 3)
        magazineOrder.add("purple");
        magazineOrder.add("purple");
        magazineOrder.add("green");

        magazineServo.setPosition(magazineShootingPositions[0]);

//        //Initialize AprilTag Detection
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                //Settings here
//                .build();
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
//                .addProcessor(aprilTagProcessor)
//                .setCameraResolution(new Size(640, 480))
//                .build();
//
//
//        //Initialization
//        magazineServo.setPosition(0.0);
//        loadServo.setPosition(0.0);
//
//        //AprilTag detection
//        try {
//            obeliskOrder = detectObelisk();
//        } catch (InterruptedException e) {
//            obeliskOrder = new String[]{"green", "purple", "purple"}; // DEFAULT ORDER FOR ISSUES
//        }
//
    }

//    @Override
//    public void init_loop(){
//        if (!gamepad2.dpad_down) { // Safe initialization mode for redundancy, holding gamepad2 dpad down while passing from init_loop() to loop() will prevent the AprilTag processor from running.
//            try { //TRY/CATCH BECAUSE IT'S REQUIRED WHEN USING SLEEP()
//                obeliskOrder = detectObelisk() == null ? detectObelisk() : obeliskOrder; //Kinda weird, will replace later, if detectObelisk() returns null, it ignores it and keeps whatever the last value was.
//            } catch (InterruptedException e) {
//                obeliskOrder = new String[]{"green", "purple", "purple"}; // DEFAULT ORDER FOR ISSUES RATHER THAN THROWING A RANDOM ERROR. IT'S RIGHT 33.33% OF THE TIME AT LEAST
//            }
//        }
//        updateInitTelemetry();
//    }

    @Override
    public void loop() {
        drive(); // translation and rotation
        spinIntakes(); //Spinning the intakes (duh) DISABLED UNTIL BUTTON IS BOUND
        launchArtifactManual(); //Manual artifact launching, default for now unless we cna get something crazy working.
        launchArtifactAutomatic(); 
        aim();
        updateTelemetry();
        magazineControlManual();

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
        
//        if (x + y > 0 && !movingElevator)
//        {
//          loadServo.setPosition(0.1); //Slightly off the ground
//        }
//        else if (x + y == 0 && !movingElevator)
//        {
//          loadServo.setPosition(loadServoPositions[1]);
//        }

        //Wheel-by-wheel spin calculations
        double FL = (-x + y - rotation) * speedMod;
        double FR = (-x - y - rotation) * speedMod;
        double BL = (x + y - rotation) * speedMod;
        double BR = (x - y - rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }

    public void aim()
    {
      double x = -gamepad2.left_stick_x;
      hoodServo.setPower(x);
    }

    public void magazineControlManual() {
        if (gamepad2.dpad_right) {
            magazineServo.setPosition(magazineShootingPositions[0]);
            previousMagazine = magazineShootingPositions[0];
        }
        else if (gamepad2.dpad_up) {
            magazineServo.setPosition(magazineShootingPositions[1]);
            previousMagazine = magazineShootingPositions[1];
        }
        else if (gamepad2.dpad_left) {
            magazineServo.setPosition(magazineShootingPositions[2]);
            previousMagazine = magazineShootingPositions[2];
        }

        if (gamepad2.x) {
            magazineServo.setPosition(magazineServo.getPosition() - 0.00025);
        } else if (gamepad2.b){
            magazineServo.setPosition(magazineServo.getPosition() + 0.00025);
        }

        if (previousMagazine == magazineServo.getPosition())
        {
          indexMagazine(); // Might need to do it every 10th clock cycle or something
        }
    }

    public void launchArtifactManual(){
        // change magazine order = the dpad brah
        // move elevator up = right bumper
        // move fly wheels = right trigger ya
        // setposition double is degree so like 1 is 180 and 0.5 is 90 <-- This is not true, you're probably thinking of radians since 180 is (pi) and 90 is (pi)/2. Unfortunately, our servos are [0, 1] between [0, 300] degrees
        // degrees vary cause idk them
        
        if (gamepad2.dpad_up && gamepad2.right_trigger > 0.1)
        {
          loadServo.setPosition(1);
        }
        else if (gamepad2.dpad_down)
        {
          loadServo.setPosition(-1);
        }
        else 
        {
          loadServo.setPosition(0);
        }

        if (gamepad2.right_trigger > 0.1) {
            //flyWheelA.setPower(-1);
            //flyWheelB.setPower(1);

            flyWheelA.setPower(-gamepad2.right_trigger * 0.85);
            flyWheelB.setPower(gamepad2.right_trigger * 0.85);
        } else {
            flyWheelA.setPower(0);
            flyWheelB.setPower(0);
        }
    }

    public void launchArtifactAutomatic()
    {
      ArrayList<Integer> order = buildLaunchOrder();
      if (gamepad2.right_trigger > 0.2)
      {
        if (order.contains(-1))
        {
          gamepad2.rumbleBlips(1);
        }

        if(loadServo.getPosition() == loadServoPositions[0])
        {
          magazineServo.setPosition(order.get(0));
        }
        else if (loadServo.getPosition() == loadServoPositions[1])
        {
          loadServo.setPosition(loadServoPositions[0]);
          order.remove(0);
        }

        if (magazineServo.getPosition() == order.get(0))
        {
          loadServo.setPosition(loadServoPositions[1]);
        }
        else
        {

        }
      }
    }


    public ArrayList<Integer> buildLaunchOrder()
    { 
      ArrayList<Integer> launchOrder = new ArrayList<>(Arrays.asList(-1, -1, -1));
      for (int i = 0; i < obeliskOrder.length; i++)
      {
        for (int j = 0; j < magazineIndex.length; j++)
        {
          if ((magazineIndex[j] == obeliskOrder[i]) && !launchOrder.contains(j))
          {
            launchOrder.set(i, j);
            break;
          }
        }
      }

      return launchOrder;
    }

    public int[] buildLaunchOrderBad()
    { //Worst code I've ever written
      ArrayList<Integer> usedIndexes = new ArrayList<>();
      boolean toDoOrNot = false;
      int[] launchOrder = new int[3];

      for (int i = 0; i <= 3; i ++)
      {
        for (int j = 0; j < launchOrder.length; j++)
        {
          toDoOrNot = true;
          if (magazineIndex[j] == obeliskOrder[0])
          {
            for (int k = 0; k < usedIndexes.size(); k++)
            {
              if (usedIndexes.get(k) == j)
              {
                toDoOrNot = false;
              }
            }
            if (toDoOrNot)
            {
            launchOrder[0] = j;
            }
          }
        }

        usedIndexes.add(launchOrder[0]); //wrong 
      }

      return launchOrder;
      
    }

    //Intake control
    public void spinIntakes() {

      if(gamepad1.a)
      {
        if (!intake_doneOnce)
        {
          previousMagazine = magazineServo.getPosition();
          magazineServo.setPosition(magazineIntakePositions[0]);
          intake_doneOnce = true;
        }

        intakeB.setPower(1.0);
        intakeA.setPower(1.0);
      } else if (gamepad1.y) {
          intakeA.setPower(-1.0);
          intakeB.setPower(-1.0);
      }
      else
      {
        intakeB.setPower(0);
        intakeA.setPower(0);
        magazineServo.setPosition(previousMagazine);
        intake_doneOnce = false;
      }
    }

    //Color sensor
//    public String colorMatch(ColorSensor colorSensor, double purpleTolerance, double greenTolerance, int steps){
//        double purpleSum = 0;
//        double greenSum = 0;
//
//        for (int i = 0; i < steps; i++){ // Takes a rolling account of readings with a certain number of steps, time unbounded, to prevent one-off errors. Unlike AprilTags where the error is often blurred motion which requires a short wait time to fix, color sensors are just kinda jank so sampling right after is usually fine.
//            double red = colorSensor.getNormalizedColors().red;
//            double green = colorSensor.getNormalizedColors().green;
//            double blue = colorSensor.getNormalizedColors().blue;
//
//            double purpleMatch = Math.max(0, Math.min(red, blue) - green - Math.abs(red - blue));
//            double greenMatch = Math.max(0, green - Math.max(red, blue));
//
//            purpleSum += purpleMatch;
//            greenSum += greenMatch;
//        }
//
//        double purpleAvg = purpleSum / steps; // Determines the average percentage [0, 1] that whatever is scanned matches purple
//        double greenAvg = greenSum / steps; // Determines the average percentage [0, 1] that whatever is scanned matches green
//
//        if (purpleAvg < purpleTolerance && greenAvg < greenTolerance){
//            return "empty";
//        }
//        else if (purpleAvg > greenAvg){
//            return "purple";
//        }
//        else {
//            return "green";
//        }
//    }

    public void updateInitTelemetry() {
        telemetry.addData("Magazine Order", magazineOrder.get(0) + " " + magazineOrder.get(1) + " " + magazineOrder.get(2));
        telemetry.addData("AprilTag Read Attempts", aprilTagReadAttempts);
        telemetry.addData("Obelisk Order", obeliskOrder[0] + " " + obeliskOrder[1] + " " + obeliskOrder[2]);
        telemetry.update();
    }

    public void updateTelemetry() {
        //telemetry.addData("Magazine Order", magazineOrder.get(0) + " " + magazineOrder.get(1) + " " + magazineOrder.get(2));
        //telemetry.addData("Wheel Powers", wheelFL.getPower() + "    " + wheelFR.getPower() + "\n" + "               "
        //                                              +wheelBL.getPower() + "     " + wheelBR.getPower()); //Hopefully this serves as a bit of a better display, remove if not.

        //telemetry.addData("CSL Red", colorSensorL.getNormalizedColors().red);
        //telemetry.addData("CSL Green", colorSensorL.getNormalizedColors().green);
        //telemetry.addData("CSL Blue", colorSensorL.getNormalizedColors().blue);

      telemetry.addData("Magazine 0", magazineIndex[0]);
//      telemetry.addData("Distance 0", colorDistanceL.getDistance(DistanceUnit.CM));

      telemetry.addData("Magazine 1", magazineIndex[1]);
//      telemetry.addData("Distance 1", colorDistanceFR.getDistance(DistanceUnit.CM));

      telemetry.addData("Magazine 2", magazineIndex[2]);
//      telemetry.addData("Distance 2", colorDistanceBR.getDistance(DistanceUnit.CM));

        ArrayList<Integer> order = buildLaunchOrder();
        telemetry.addData("Order", order.get(0) + ", " + order.get(1) + ", " + order.get(2));

        telemetry.update();


    }

    public void indexMagazine()
    {
      magazineIndex[0] = guessColorL(colorSensorL.red(),colorSensorL.green(),colorSensorL.blue());
      magazineIndex[1] = guessColorFR(colorSensorFR.red(),colorSensorFR.green(),colorSensorFR.blue());
      magazineIndex[2] = guessColorBR(colorSensorBR.red(),colorSensorBR.green(),colorSensorBR.blue());

    }


    public String guessColorL(int red, int green, int blue) {
        int total = red + green + blue;
        String answer = "";
        if (total > 250 && (colorDistanceL.getDistance(DistanceUnit.CM) < 20)) {
            if ((green - blue > (total * .04)) && (green - blue > 0)) {
                answer = "green";
            }
            else if (blue - green > (total * .015)) {
                answer = "purple";
            } else {
                answer = "none";
            }
        }
        else
        {
            answer = "none";
        }
        return answer;
    }

    public String guessColorBR(int red, int green, int blue) {
        int total = red + green + blue;
        String answer = "";
        if (total > 150 && (colorDistanceBR.getDistance(DistanceUnit.CM) < 4.5)) {
            if ((green - blue > (total * .02)) && (green - blue > 0)) {
                answer = "green";
            }
            else if (blue - green > (total * .01)) {
                answer = "purple";
            } else {
                answer = "none";
            }
        }
        else
        {
            answer = "none";
        }
        return answer;
    }

    public String guessColorFR(int red, int green, int blue) {
        int total = red + green + blue;
        String answer = "";
        if (total > 150 && (colorDistanceFR.getDistance(DistanceUnit.CM) < 4)) {
            if ((green - blue > (total * .02)) && (green - blue > 0)) {
                answer = "green";
            }
            else if (blue - green > (total * .01)) {
                answer = "purple";
            } else {
                answer = "none";
            }
        }
        else
        {
            answer = "none";
        }
        return answer;
    }
}

    // Scans the obelisk, returns either null for "can't see anything" and "ID out of range" or a 3-value String[] if it identifies an appropriate AprilTag code
//    public String[] detectObelisk() throws InterruptedException {
//        ArrayList<AprilTagDetection> obeliskDetections = aprilTagProcessor.getDetections(); //Gets the first detection of an apriltag, should only be the center one
//        if (obeliskDetections.isEmpty()) {
//            return null;
//        } else {
//            int id = obeliskDetections.get(0).id; //Gets the first element, only ONE element should be in frame.
//            switch (id) {
//                case 21:
//                    return new String[]{"green", "purple", "purple"};
//                case 22:
//                    return new String[]{"purple", "green", "purple"};
//                case 23:
//                    return new String[]{"purple", "purple", "green"};
//                default:
//                    return null; //REFERS TO DEFAULT FAILURE HANDLING IN LOOP
//            }
//        }

//    }

//}
