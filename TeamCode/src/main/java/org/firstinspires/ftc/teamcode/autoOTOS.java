/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
/* this is an autonomous program for red. Start centered on tile F2 along wall.
    It uses the SparkFun OTOS sensor to control driving.
    It drives forward to push a sample into the net zone,
    then moves to tile E2, then backs up to E6 before parking in the 
    red observation zone. 
*/

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Final Auto", group="auto")
//@Disabled
public class autoOTOS extends LinearOpMode {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN  =  0.25;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  0.175;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.4   ;   //  Clip the turn speed to this max value (adjust for your robot)

    private ElapsedTime runtime = new ElapsedTime();
    
    // Declare OpMode members for each of the 4 drive motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor drawer = null;
    private DcMotor speciman = null;
    private SpecimenClaw claw = null;
    private Claw fisher = null;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 * 10.5 / 9.5;
    // ^ GoBILDA 5203 Series Yellow Jacket Planetary Gear Motor specs, check store page for motor to find number, CPR is PPR * 4
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // use ruler to measure diameter of whatever wheel
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     SPECIMAN_SPEED             = 0.35; // max power for speciman arm
    static final double     DRAWER_SPEED             = 0.35; // max power for drawer arm

    //public static double Linear_Scaler = 20.281/21 * 48.3115/56.0 * 27.5/23.36;
    public static double Linear_Scaler = 0.98082701825;
    public static double Angular_Scaler = 1.000;
    
    // Sensors
    private SparkFunOTOS myOtos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() throws InterruptedException {

        String LEFT_FRONT_DRIVE_NAME = "leftFrontDrive";
        String RIGHT_FRONT_DRIVE_NAME = "rightFrontDrive";
        String LEFT_REAR_DRIVE_NAME = "leftRearDrive";
        String RIGHT_REAR_DRIVE_NAME = "rightRearDrive";
        String DRAWER_NAME = "drawerMotor";
        String SPECIMAN_NAME = "specimanMotor";
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.dcMotor.get(LEFT_FRONT_DRIVE_NAME);
        leftBackDrive  = hardwareMap.dcMotor.get(LEFT_REAR_DRIVE_NAME);
        rightFrontDrive = hardwareMap.dcMotor.get(RIGHT_FRONT_DRIVE_NAME);
        rightBackDrive = hardwareMap.dcMotor.get(RIGHT_REAR_DRIVE_NAME);
        drawer = hardwareMap.dcMotor.get(DRAWER_NAME);
        speciman = hardwareMap.dcMotor.get(SPECIMAN_NAME);
        claw = new SpecimenClaw(hardwareMap);
        fisher = new Claw(hardwareMap);

        //setting directions, on current robot any axle pointing left is reversed, pointing right is forward
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        drawer.setDirection(DcMotor.Direction.REVERSE);
        speciman.setDirection(DcMotor.Direction.REVERSE);

        drawer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speciman.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drawer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        speciman.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFun");
        pos = myOtos.getPosition();
        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        sleep(1000);
        

        while(!isStarted()) {
            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        waitForStart();
        runtime.reset();
       
        telemetry.addData("Status", "Running");
        telemetry.update();

        /*
        while(opModeIsActive()){
            pos = myOtos.getPosition();
            telemetry.addData("Cords:", "X %5.2f, Y %5.2f, Rotation %5.2f", pos.x, pos.y, pos.h);
            telemetry.update();
        }
        */

        //drawerArm(0.5, 15, 5);
        claw.setClawJoint(0.3146);
        claw.grabSpecimen();
        otosDrive(23, 5, 4, 2);
        otosRotate(-45, 4, 4);

        fisher.setClawPosition(0);
        drawerArm(0.84, 4, 2);
        fisher.setClawPosition(0.8);
        fisher.setJointPosition(0.26);
        sleep(500); // deposit preload
        //reportPos();
        otosRotate(35, 2.5, 4);
        sleep(500);
        otosForward(10, 5, 1.5);
        drawerArm(0.84, 8, 3.5);//claw around 1st ground sample
        drawerArm(0.84, -2, 1.5);
        fisher.setJointPosition(0);
        sleep(400);
        fisher.setClawPosition(0);
        drawerArm(0.84, -12, 3);
        drawer.setPower(-0.2);
        fisher.setJointPosition(0.26);
        SPEED_GAIN /= 1.5;
        otosForward(1.5, 2.5, 1.5);
        otosRotate(-35, 3, 4);
        fisher.setClawPosition(0.4);
        sleep(500);

        otosRotate(35, 2.5, 4);
        sleep(500);
        otosForward(7.5, 5, 1.5);
        otosRotate(22.5, 3, 4);
        fisher.setClawPosition(0.8);
        drawerArm(0.84, 14, 3.5);//claw around 2nd ground sample
        fisher.setJointPosition(0);
        sleep(400);
        fisher.setClawPosition(0);
        drawerArm(0.84, -16, 3);
        drawer.setPower(-0.2);
        fisher.setJointPosition(0.26);
        otosRotate(-22.5, 2.5, 4);
        otosForward(2.5, 1.5, 1.5);
        otosRotate(-35, 2.5, 4);
        //otosRotate(-35, 3, 4);
        fisher.setClawPosition(0.4);
        sleep(500);
        //reportPos();

        otosRotate(37, 2.5, 4);
        otosForward(7.5, 4, 1.5);
        otosRotate(45, 2.5, 4);
        drawerArm(1, 4, 2);
        fisher.setJointPosition(0.26);
        fisher.setClawPosition(0.8);
        drawerArm(1, 11.25, 3.5);//claw around 2nd ground sample
        fisher.setJointPosition(0);
        sleep(350);
        drawerArm(1, -4, 1);
        fisher.setClawPosition(0);
        drawerArm(1, -11.25, 3.5);
        fisher.setJointPosition(0.26);
        //sleep(20000);
        otosRotate(-58, 4, 4);
        otosForward(4.5, 2, 1.5);
        otosRotate(-15, 2, 4);

        /*
        auto without threads

        claw.grabSpecimen();
        otosDrive(23, 5, 4, 2);
        otosRotate(-45, 4, 4);

        fisher.setClawPosition(0);
        drawerArm(0.84, 4, 2);
        fisher.setClawPosition(0.8);
        fisher.setJointPosition(0.26);
        sleep(500); // deposit preload
        //reportPos();
        otosRotate(35, 2.5, 4);
        sleep(500);
        otosForward(10, 5, 1.5);
        drawerArm(0.84, 8, 3.5);//claw around 1st ground sample
        drawerArm(0.84, -2, 1.5);
        fisher.setJointPosition(0);
        sleep(400);
        fisher.setClawPosition(0);
        drawerArm(0.84, -12, 3);
        drawer.setPower(-0.2);
        fisher.setJointPosition(0.26);
        SPEED_GAIN /= 1.5;
        otosForward(1.5, 2.5, 1.5);
        otosRotate(-35, 3, 4);
        fisher.setClawPosition(0.4);
        sleep(500);

        otosRotate(35, 2.5, 4);
        sleep(500);
        otosForward(7.5, 5, 1.5);
        otosRotate(22.5, 3, 4);
        fisher.setClawPosition(0.8);
        drawerArm(0.84, 14, 3.5);//claw around 2nd ground sample
        //drawerArm(0.84, -2, 1.5);
        fisher.setJointPosition(0);
        sleep(400);
        fisher.setClawPosition(0);
        drawerArm(0.84, -16, 3);
        drawer.setPower(-0.2);
        fisher.setJointPosition(0.26);
        //SPEED_GAIN /= 1.5;
        otosRotate(-22.5, 2.5, 4);
        otosForward(2.5, 1.5, 1.5);
        otosRotate(-35, 2.5, 4);
        //otosRotate(-35, 3, 4);
        fisher.setClawPosition(0.4);
        sleep(500);
        //reportPos();

        otosRotate(37, 2.5, 4);
        otosForward(7.5, 4, 1.5);
        otosRotate(45, 2.5, 4);
        drawerArm(1, 4, 2);
        fisher.setJointPosition(0.26);
        fisher.setClawPosition(0.8);
        drawerArm(1, 11.25, 3.5);//claw around 2nd ground sample
        //drawerArm(1, -1, 3.5);
        //drawerArm(0.84, -2, 1.5);
        fisher.setJointPosition(0);
        sleep(350);
        drawerArm(1, -4, 1);
        fisher.setClawPosition(0);
        drawerArm(1, -11.25, 3.5);
        fisher.setJointPosition(0.26);
        //sleep(20000);
        otosRotate(-58, 4, 4);
        otosForward(4.5, 2, 1.5);
        otosRotate(-15, 2, 4);
         */
        //reportPos();
        //otosDrive(4.83, 7.6, 3.5, 1.5);
        //otosDrive(2, myPosition().y+1, 3.5, 1.5);
        //otosForward(2, 3 ,1.5);
        //specimanArm(0.84, 16, 5, 0.2);
        //otosForward(15, 6, 2); // drive to bar
        //otosDrive(15, 15, 6, 2);
        //otosDrive(0, 0 ,6, 2);

        //specimanArm(0.84, -16, 2.5, 0);
        //claw.setClawJoint(0.2);
        //speciman.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //speciman.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clearAll();
        //long x = runtime.time(TimeUnit.MILLISECONDS);
        //speciman.setPower(-0.5);
        //sleep(1600);
        /*
        while (2000 + x > runtime.time(TimeUnit.MILLISECONDS)) {
            speciman.setPower(-0.4);
            telemetry.addData("Motor Power: ", speciman.getPower());
            telemetry.update();
            //specimanArm(0.84, -6, 3, -0.3);
        }
         */
        claw.releaseSpecimen();

        //x = runtime.time(TimeUnit.MILLISECONDS);
        /*
        while (2000 + x > runtime.time(TimeUnit.MILLISECONDS)) {

            speciman.setPower(-0.4);
            telemetry.addData("Motor Power: ", speciman.getPower());
            telemetry.update();
            //specimanArm(0.84, -6, 3, -0.3);
        }
         */
        //moveRobot(0, -0.4, 0);
        //sleep(1000);
        //otosForward(15,3.5, 2); // drive back
        //otosDrive(0, 15, -117.41, 100, 0.1);
        //otosRotate(-180, 4, 4); // rotate
        STRAFE_GAIN = 0.4;
        SPEED_GAIN = 0.05;
        speciman.setPower(0);
        //sleep(50000);
        //otosDrive(-39.41, 3.33, 5, 0.2);
        //otosDrive(3.33, -39.41, 5, 0.2);
        //otosDrive(40.7, 1.45, 6.5, 0.1);
        //otosDrive(-40.7, 2.45, 100, 0.1); // drive to zone

        //otosDrive(-40.94, 0.35, -177.41, 100, 0.1);
        /*
        while (opModeIsActive()) {
            speciman.setPower(0.27);
        }
         */
        //otosDrive(0,48, 0, 1000, 0.2);
        /*
        otosDrive(2, 2, 0, 2);      // small moveforward and right away from wall
        otosDrive(18, 2, 0, 2);     // forward and push sample into net zone
        
        otosDrive(0, 24, 0, 2);     // backup and move away from wall
        otosDrive(-87, 24, 0, 4);   // backup straight
        otosDrive(-87, 4, 0, 2);    // park in observation zone
        
        sleep(1000);
        */
    }
    
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(DistanceUnit.CM);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0); // inches, inches, degrees
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(Linear_Scaler);
        myOtos.setAngularScalar(Angular_Scaler);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    /**
     * Move robot to a designated X,Y position and heading (rotation)
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
     void otosDrive(double targetX, double targetY, double maxTime, double error) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x; 
        yError = targetY-currentPos.y;
        yawError = 0;
        
        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
        ((Math.abs(xError) > error) || (Math.abs(yError) > error) || (Math.abs(yawError) > 4)) ) {
        // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * -STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();
                
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, -strafe, turn);
            
            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;  
            yError = targetY-currentPos.y;
            yawError = 0;
        }
        moveRobot(0,0,0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    /**
     * Move robot to a designated X,Y position, heading (rotation), and how close to that position
     * set the maxTime to have the driving logic timeout after a number of seconds.
     * set error to have the robot move within that many DistanceUnits of given location
     */
    void otosDrive(double targetX, double targetY, double targetHeading, double maxTime, double error) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
                ((Math.abs(xError) > error) || (Math.abs(yError) > error) || (Math.abs(yawError) > 4)) ) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = -targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    void otosForward(double targetY, double maxTime, double error) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        double rotation = myPosition().h;
        SparkFunOTOS.Pose2D currentPos = new SparkFunOTOS.Pose2D(myPosition().x, myPosition().y, 0);
        yError = targetY-currentPos.y;
        //yError = targetY-currentPos.y;
        //yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
                (Math.abs(yError) > error)) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            //strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            //turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f", drive);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetY);
            //telemetry.addData("target Y coordinate", targetY);
            //telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", yError);
            //telemetry.addData("yError", yError);
            //telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(0, drive,  0);

            // then recalc error
            currentPos = myPosition();
            yError = targetY-currentPos.y;
            //yError = targetY-currentPos.y;
            //yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        currentPos = new SparkFunOTOS.Pose2D(myPosition().x, myPosition().y, rotation);
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    void otosRotate(double targetHeading, double maxTime, double error) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        //xError = targetX-currentPos.x;
        //yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
                (Math.abs(yawError) > error)) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            //drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            //strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Turn %5.2f ", turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            //telemetry.addData("target X coordinate", targetX);
            //telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            //telemetry.addData("xError", xError);
            //telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(0, 0, turn);

            // then recalc error
            currentPos = myPosition();
            //xError = targetX-currentPos.x;
            //yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        myOtos.setPosition(new SparkFunOTOS.Pose2D(myPosition().x, myPosition().y, yawError));
        myOtos.setOffset(new SparkFunOTOS.Pose2D(0, 0, currentPos.h));
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
    SparkFunOTOS.Pose2D myPosition() {
        pos = myOtos.getPosition();
        SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(pos.x, pos.y, -pos.h);
        return(myPos);
    }
    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive X is forward
     * Positive Y is strafe right
     * Positive Yaw is clockwise: note this is not how the IMU reports yaw(heading)
     */
     /*
     void moveRobot(double y, double x, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        //screw with the pause timing to see what happens
        sleep(10);
    }
      */

    void moveRobot(double x, double y, double turn){ //right, forward, turn

        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rightX = turn;

        double leftFrontWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) + turn;
        double rightFrontWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) - turn;
        double leftRearWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) + turn;
        double rightRearWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) - turn;

        leftFrontDrive.setPower(leftFrontWheelPower);
        rightFrontDrive.setPower(rightFrontWheelPower);
        leftBackDrive.setPower(leftRearWheelPower);
        rightBackDrive.setPower(rightRearWheelPower);


        if (false) {
            telemetry.addData("leftFront", "%.2f", leftFrontWheelPower);
            telemetry.addData("rightFront", "%.2f", rightFrontWheelPower);
            telemetry.addData("leftRear", "%.2f", leftRearWheelPower);
            telemetry.addData("rightRear", "%.2f", rightRearWheelPower);
            telemetry.addData("desiredAxis", "%.2f", rightX);
        }
    }

    //arm extension modelled of off "RobotAutoDriveByEncoder_Linear"'s encoder drive function
    public void drawerArm(double speed, double extension, double timeoutS) {
        int target;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = drawer.getCurrentPosition() + (int)(extension * COUNTS_PER_INCH);
            drawer.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            drawer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            drawer.setPower(Math.abs(speed * DRAWER_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && drawer.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", target);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        drawer.getCurrentPosition(), drawer.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            drawer.setPower(0);

            // Turn off RUN_TO_POSITION
            drawer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10);   // optional pause after each move.
        }
    }

    public void specimanArm(double speed, double extension, double timeoutS, double endPower) {
        int target;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = speciman.getCurrentPosition() + (int)(extension * COUNTS_PER_INCH);
            speciman.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            speciman.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            speciman.setPower(Math.abs(speed * SPECIMAN_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && speciman.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", target);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        speciman.getCurrentPosition(), speciman.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            speciman.setPower(endPower);

            // Turn off RUN_TO_POSITION
            speciman.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(10);   // optional pause after each move.
        }
    }
    void reportPos(){
        while(opModeIsActive()){
            pos = myOtos.getPosition();
            telemetry.addData("Cords:", "X %5.2f, Y %5.2f, Rotation %5.2f", pos.x, pos.y, pos.h);
            telemetry.update();
        }
    }
}