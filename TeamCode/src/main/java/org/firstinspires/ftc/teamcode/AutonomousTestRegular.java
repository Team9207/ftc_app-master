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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.SynchronousQueue;

/**
 * WHEEL DIAMETER = 4.7 IN
 * CIRCUMFERENCE = 14.8 IN
 * 420 TICKS = 1 ROTATION
 * 342 TICKS = ~1 FT
 */
@Autonomous(name = "Autonomous Test Regular", group = "Autonomous")

public class AutonomousTestRegular extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BLeftMotor = null;
    private DcMotor BRightMotor = null;
    private DcMotor lyftdrive = null;
    private DcMotor suckDrive = null;
    private DcMotor armDrive = null;
    private Servo servo1 = null;

    public void MoveForward(double feet) {
        BLeftMotor.setTargetPosition((int) (feet * 342));
        BRightMotor.setTargetPosition((int) (feet * 342));
    }
    @Override
    public void init() {
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

        BLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BRightMotor.setDirection(DcMotor.Direction.FORWARD);
        lyftdrive.setDirection(DcMotor.Direction.REVERSE);
        suckDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        //Sets up motors to run to encoder value specified with [DcMotor].setTargetPosition();
        BLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
public void turnRight(double deg){
    //Code to make the robot turn
    // I found this website https://robotstpjhs.wikispaces.com/Calculating+Turns
}
public void turnLeft(double deg){
    turnRight(-deg);
    //this just turns a negative amount to the right which is left.
    //throwing negative values into turnRight() is the same thing but this looks nicer
}
    @Override   //Runs once when the OpMode is selected
    public void init_loop() {

    }
    @Override   //Runs once when the start button is pressed
    public void start(){
        runtime.reset();
    }
    @Override   //Loops until stopped
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

}
