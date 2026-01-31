
package org.firstinspires.ftc.teamcode;
//IMPORTS
import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp(name = "getMagazinePos", group = "Axolotl")
public class getMagazinePos extends OpMode {

    //Hardware
    private Servo magazineServo; //Servos
    public double posCurrent = 0;
    public double pos1;
    public double pos2;
    public double pos3;

    @Override
    public void init() {
        magazineServo = hardwareMap.get(Servo.class, "magazineServo");
        magazineServo.setPosition(posCurrent);
    }


    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.1) {
            if (gamepad1.left_bumper) {
                posCurrent = posCurrent - 0.000001;
            }
            if (gamepad1.right_bumper) {
                posCurrent = posCurrent + 0.000001;
            }
        } else {
            if (gamepad1.left_bumper) {
                posCurrent = posCurrent - 0.0001;
            }
            if (gamepad1.right_bumper) {
                posCurrent = posCurrent + 0.0001;
            }
        }

        if (gamepad1.a)
        {
          magazineServo.setPosition(posCurrent);
        }


        if (gamepad1.dpad_left) {
            pos1 = posCurrent;
        }
        if (gamepad1.dpad_up) {
            pos2 = posCurrent;
        }
        if (gamepad1.dpad_right) {
            pos3 = posCurrent;
        }


        telemetry.addData("Current Position: ", posCurrent);
        telemetry.addData("Position One: ", pos1);
        telemetry.addData("Position Two: ", pos2);
        telemetry.addData("Position Three: ", pos3);
        telemetry.update();
    }
}
