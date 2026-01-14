package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

@TeleOp (name = "mallardDrive", group = "Axolotl")
public class mallardDrive extends OpMode {

    //Setting variables for motors and servos
    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR, intakeB;
    private Servo elevate, rotate;
    int readings;
    private DistanceSensor distanceSensorFront, distanceSensorBack;

    NormalizedColorSensor colorSensorLeft, colorSensorRight;

    // Color sensor variables
    private final ColorMatch colorMatcher = new ColorMatch();

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

    
    public void init() {
        telemetry.addData("Status", "Initialization Started");

        //Movement wheels initialization
        //   wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        //   wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        //   wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        //   wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        //Intake initialization

        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");

        //Magazine servos initialization
        //    elevate = hardwareMap.get(Servo.class, "elevate");
        //    rotate = hardwareMap.get(Servo.class, "rotate");

        //Distance sensor initialization
        //distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceSensorFront");
        //distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");

        //Color sensor initialization
        //colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        colorSensorLeft.setGain(35);

        //colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");
	colorSensorRight  = hardwareMap.get(ColorSensorV3.class, "colorSensorRight");
        colorSensorRight.setGain(35);

        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);


        telemetry.addData("Status", "Initialized and Ready");

    }

    // loop variables
    int distanceCycle = 0;
    double distanceFront;
    double distanceBack;
    double[] arrayFront = new double[10];
    double[] arrayBack = new double[10];

    @Override
    //Methods that will be called and be forever running during teleop
    public void loop() {
        // drive();
        // ariseElevator();
        // rotationManual();
        //avgDistances();
        getDetectedColor();
        getDetectedColorRight();
        //rotationAutomatic();
        //launch();
        spinIntakes();
    }

    //Methods
    public void drive() {
        //Setting variables for x and y (and rotation) axis positions on controller sticks
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;

        //Speed that motors run (half the speed they would originally run)
        double speedMod = 0.5;
        //Variables that calculate the speed each wheel goes (allows for turning and strafing)
        double FL = ( x + y + rotation) * speedMod;
        double FR = ( x - y + rotation) * speedMod;
        double BL = ( x - y - rotation) * speedMod;
        double BR = ( -x - y + rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }

    public void spinIntakes() {
        intakeB.setPower(0.5);
    }
    public void ariseElevator() {
        // ascendButton = gamepad1.y;
        // descendButton = gamepad1.a;

        if (gamepad1.y) {
            elevate.setPosition(0.1);
        } else if (gamepad1.a) {
            elevate.setPosition(0.9);
        } else {
            elevate.setPosition(0.5);
        }
    }


    public void avgDistances()
    {
        DistanceSensor dsb = distanceSensorBack;
        DistanceSensor dsf = distanceSensorFront;

        arrayFront[distanceCycle] = dsf.getDistance(DistanceUnit.MM);
        arrayBack[distanceCycle] = dsb.getDistance(DistanceUnit.MM);

        distanceCycle++;

        if (distanceCycle > 9) { distanceCycle = 0; }

        distanceFront = getAvg(arrayFront);
        distanceBack = getAvg(arrayBack);

        telemetry.addData("Distance Front: ", distanceFront + "mm");
        telemetry.addData("Distance Back: ", distanceBack + "mm");
        telemetry.addData("Distance no algorithm Front: ", distanceSensorFront.getDistance(DistanceUnit.MM) + "mm");
        telemetry.addData("Distance no algorithm Back: ", distanceSensorBack.getDistance(DistanceUnit.MM) + "mm");

    }

    public double getAvg(double[] list)
    {
        int sum = 0;
        for (int i = 0; i < list.length; i++)
        {
            sum += list[i];
        }
        return sum / list.length;
    }
    public void rotationManual(){
        // rotateButton = gamepad1.b;

//        telemetry.addData("deviceName", distanceSensorFront.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", (distanceSensorBack));
//        telemetry.addData("deviceName", distanceSensorBack.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", distanceSensorBack.getDistance(DistanceUnit.MM)));

        if (gamepad1.b){
            rotate.setPosition(0.9);
        } else if (distanceSensorFront.getDistance(DistanceUnit.MM) > 0) {
            rotate.setPosition(0.0);
        }
    }
    public void launch(){

        // launchTrigger = gamepad1.right_trigger;

    }
    public String getDetectedColor(){
        //return 4 values in percentages (red, green, blue, alpha(brightness))
        NormalizedRGBA colors = colorSensorLeft.getNormalizedColors();
        //colors = colorSensorRight.getNormalizedColors();

        float leftRed, leftGreen, leftBlue;
        leftRed = colors.red / colors.alpha;
        leftGreen = colors.green / colors.alpha;
        leftBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", leftRed);
        telemetry.addData("Green", leftGreen);
        telemetry.addData("Blue", leftBlue);

        // purple = .2142, .2683, .366
        // green = .1275, .4153, .3023

        if (leftRed > .20 && leftGreen < .27 && leftBlue > .36){
            telemetry.addData("Color detected LEFT", "PURPLE");
	    return "purple";
        } else if (leftRed < .13 && leftGreen > .4 && leftBlue < .34){
            telemetry.addData("Color detected LEFT", "GREEN");
	    return "green";
        } else {
            telemetry.addData("Color detected LEFT", "NONE");
	    return "other";
        }

    }

    public String getDetectedColorRight(){
        //return 4 values in percentages (red, green, blue, alpha(brightness))
        NormalizedRGBA colors = colorSensorRight.getNormalizedColors();
        //colors = colorSensorRight.getNormalizedColors();

        float rightRed, rightGreen, rightBlue;
        rightRed = colors.red / colors.alpha;
        rightGreen = colors.green / colors.alpha;
        rightBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", rightRed);
        telemetry.addData("Green", rightGreen);
        telemetry.addData("Blue", rightBlue);

        // purple = .2142, .2683, .366
        // green = .1275, .4153, .3023

        if (rightRed > .20 && rightGreen < .27 && rightBlue > .36){
            telemetry.addData("Color detected RIGHT", "PURPLE");
	    return "purple";
        } else if (rightRed < .13 && rightGreen > .4 && rightBlue < .34){
            telemetry.addData("Color detected RIGHT", "GREEN");
	    return "green";
        } else {
            telemetry.addData("Color detected RIGHT", "NONE");
	    return "other";
        }
    }

    public String getColorOmar( ColorSensorV3 funcSensor )
    {
      Color detectedColor = funcSensor.getColor();
      String colorString;
      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

      if (match.color == kBlueTarget) {
        colorString = "Blue";
      }
      else if (match.color == kRedTarget) {
        colorString = "Red";
      } 
      else if (match.color == kGreenTarget) {
        colorString = "Green";
      } 
      else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } 
      else {
        colorString = "Unknown";
      }


    telemetry.addData.putNumber("Red", detectedColor.red);
    telemetry.addData.putNumber("Green", detectedColor.green);
    telemetry.addData.putNumber("Blue", detectedColor.blue);
    telemetry.addData.putNumber("Confidence", match.confidence);
    telemetry.addData.putString("Detected Color", colorString);

    }

}
