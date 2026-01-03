package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "mallardDrive", group = "Axolotl")
public class mallardDrive extends OpMode {

    //Setting variables for motors and servos
    private DcMotorEx wheelFL, wheelFR, wheelBL, wheelBR;
    private Servo elevate, rotate;
    private DistanceSensor distanceSensorFront, distanceSensorBack;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialization Started");

        //Movement wheels initialization
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        //Magazine servos initialization
        elevate = hardwareMap.get(Servo.class, "elevate");
        rotate = hardwareMap.get(Servo.class, "rotate");

        //Distance sensor initialization
        distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceUno");
        distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceDos");

        telemetry.addData("Status", "Initialized and Ready");

	// loop variables
	int distanceCycle = 0;
	int distanceFront;
	int distanceBack;
	int[100] arrayFront;
	int[100] arrayFront;
    }

    @Override
    //Methods that will be called and be forever running during teleop
    public void loop() {
        drive();
        ariseElevator();
        rotationManual();
	avgDistances;
        //rotationAutomatic();
        //launch();
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
        double FL = (-x - y - rotation) * speedMod;
        double FR = (-x + y - rotation) * speedMod;
        double BL = ( x - y - rotation) * speedMod;
        double BR = ( x + y - rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);
    }
    public void ariseElevator() {
        // ascendButton = gamepad1.y;

        if (gamepad1.y){
            elevate.setPosition(0.5);
        } else {
            elevate.setPosition(0.0);
        }
    }

    public void avgDistances()
    {
      dsb = distanceSensorBack;
      dsf = distanceSensorFront;

      arrayFront[distanceCycle] = dsf.getDistance(DistanceUnit.MM);
      arrayBack[distanceCycle] = dsb.getDistance(DistanceUnit.MM);

      distanceCycle++;

      if (distanceCycle > 99) { distanceCycle = 0 } 

      distanceFront = getAvg(arrayFront);
      distanceBack = getAvg(arrayBack);

      telemetry.addData("Distance Front: ", distanceFront);
      telemetry.addData("Distance Back: ", distanceBack);
    }

    public double getAvg(int[] list)
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

        telemetry.addData("deviceName", distanceSensorFront.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", (distanceSensorBack));
        telemetry.addData("deviceName", distanceSensorBack.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", distanceSensorBack.getDistance(DistanceUnit.MM)));

        if (gamepad1.b){
            rotate.setPosition(0.5);
        } else if (distanceSensorFront.getDistance(DistanceUnit.MM) > 0) {
            rotate.setPosition(0.0);
        }
    }
    public void launch(){
        // launchTrigger = gamepad1.right_trigger;

    }

}

