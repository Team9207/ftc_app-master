/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Regular Drive", group="Linear Opmode")
//@Disabled
public class Driver2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BLeftMotor = null;
    private DcMotor BRightMotor = null;
    private DcMotor lyftdrive = null;
    private DcMotor suckDrive = null;
    private DcMotor armDrive = null;
    private Servo servo1 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BLeftMotor = hardwareMap.get(DcMotor.class, "BleftMotor");
        BRightMotor = hardwareMap.get(DcMotor.class, "BrightMotor");
        lyftdrive = hardwareMap.get(DcMotor.class, "lyft_motor");
        suckDrive = hardwareMap.get(DcMotor.class, "suck_motor");
        armDrive = hardwareMap.get(DcMotor.class, "arm_motor");
        servo1 = hardwareMap.get(Servo.class, "Servo1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        BLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BRightMotor.setDirection(DcMotor.Direction.FORWARD);
        lyftdrive.setDirection(DcMotor.Direction.REVERSE);
        suckDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        lyftdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double lyftPowerUp;
            double lyftPowerDown;
            double suckPowerOn = 0.0;
            double armPowerDown;
            double armPowerUp;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double lyftUp = gamepad1.left_trigger;
            double lyftDown = gamepad1.right_trigger;
            boolean suckOn = gamepad1.y;
            boolean armDown = gamepad1.right_bumper;
            boolean armUp = gamepad1.left_bumper;


            leftPower = Range.clip(Math.pow((drive + turn), 3), -1.0, 1.0);
            rightPower = Range.clip(Math.pow((drive - turn), 3), -1.0, 1.0);
            lyftPowerUp = Range.clip(lyftUp, 0.0, 1.0);
            lyftPowerDown = Range.clip(lyftDown, 0.0, 1.0);


            if (gamepad1.a) {
                suckDrive.setPower(1);
            }
            if (!gamepad1.a) {
                suckDrive.setPower(0);
            }
            if (gamepad1.right_bumper) {
                armDrive.setPower(1.0);
            }
            if (!gamepad1.right_bumper) {
                armDrive.setPower(0);
            }
            if (gamepad1.left_bumper) {
                armDrive.setPower(-1.0);
            }
            if (!gamepad1.left_bumper) {
                armDrive.setPower(0);
            }
            if (gamepad1.y) {
                servo1.setPosition(-1);
            } else {
                servo1.setPosition(1);


                BLeftMotor.setPower(leftPower);
                BRightMotor.setPower(rightPower);
                lyftdrive.setPower(lyftPowerUp - lyftPowerDown);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }
    }
}