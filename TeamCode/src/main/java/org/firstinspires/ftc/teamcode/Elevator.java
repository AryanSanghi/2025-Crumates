package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator {

    private String ELEVATOR_MOTOR_NAME = "elevatorMotor";

    private double ELEVATOR_MOTOR_POWER = 0.8;

    private DcMotor elevatorMotor = null;
    private Bucket bucket = null;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.dcMotor.get(ELEVATOR_MOTOR_NAME);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bucket = new Bucket(hardwareMap);
    }

    public void update(Gamepad gamepad) {
        update(gamepad.right_stick_y * 0.85);
        bucket.update(gamepad);
    }

    public void update(double power) {
        elevatorMotor.setPower(power);
    }
}
