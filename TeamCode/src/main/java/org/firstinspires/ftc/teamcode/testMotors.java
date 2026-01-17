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

@TeleOp (name = "testMotors", group = "Axolotl")
public class testMotors extends OpMode {

    //Setting variables for motors and servos
    private DcMotorEx flyL, flyR;
    public boolean toggle = false;

    public void init() {
        telemetry.addData("Status", "Initialization Started");

        flyL = hardwareMap.get(DcMotorEx.class, "flyL");
        flyR = hardwareMap.get(DcMotorEx.class, "flyR");

        telemetry.addData("Status", "Initialized and Ready");

    }

    // loop variables

    @Override
    //Methods that will be called and be forever running during teleop
    public void loop() {
        spinFlywheels();
    }

    //Methods
    public void spinFlywheels() {
      if (gamepad1.y)
      {
          if (toggle)
          {
              toggle = false;
          }
          else
          {
              toggle = true;
          }

      }
      if (toggle)
      {
        flyL.setPower(1);
        flyR.setPower(-1);
      }
      else
      {
        flyL.setPower(0);
        flyR.setPower(0);
      }
    }}
