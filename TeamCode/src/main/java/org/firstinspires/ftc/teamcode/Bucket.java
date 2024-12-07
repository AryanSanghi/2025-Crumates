package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bucket {
    private String SERVO_NAME = "bucketServo";

    private Servo servo;
    private boolean servoTriggered, yDown;

    public Bucket(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get(SERVO_NAME);
        servoTriggered = false;
        yDown = false;
    }

    public void update(Gamepad gamepad){

        if(gamepad.dpad_up){
            servo.setPosition(0.8);
        } else if(gamepad.dpad_down){
            servo.setPosition(0);
            servoTriggered = !servoTriggered;
        }
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);
    }
}
