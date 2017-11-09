

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Scott's Mecanum Drive", group="Bot 1")
//@Disabled
public class Op_Mecanum_Drive_Backup extends OpMode
{
    //CONSTANTS
    public static final String LEFTDRIVE_F_MAP = "leftMotorF";                     //Constant for Hardware Map
    public static final String LEFTDRIVE_B_MAP = "leftMotorB";                     //Constant for Hardware Map
    public static final String RIGHTDRIVE_F_MAP = "rightMotorF";                   //Constant for Hardware Map
    public static final String RIGHTDRIVE_B_MAP = "rightMotorB";                   //Constant for Hardware Map
    public static final String LIFT_MOTOR = "liftMotor";
    public static final String CLAW_MOTOR = "liftMotor";
    public static final Double EXPO = 5.0;                                         //Constant for Expo value for control


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveF = null;
    private DcMotor leftDriveB = null;
    private DcMotor rightDriveF = null;
    private DcMotor rightDriveB = null;
    private DcMotor liftDrive = null;
    private DcMotor clawDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()                                                          //Loops after INIT press
    {
        telemetry.addData("Status", "Initialized");

        leftDriveF  = hardwareMap.get(DcMotor.class, LEFTDRIVE_F_MAP);
        leftDriveB  = hardwareMap.get(DcMotor.class, LEFTDRIVE_B_MAP);
        rightDriveF = hardwareMap.get(DcMotor.class, RIGHTDRIVE_F_MAP);
        rightDriveB = hardwareMap.get(DcMotor.class, RIGHTDRIVE_B_MAP);
        liftDrive = hardwareMap.get(DcMotor.class, LIFT_MOTOR);
        clawDrive = hardwareMap.get(DcMotor.class, CLAW_MOTOR);

        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);
        liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        clawDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Hardware Ready");
    }

    @Override
    public void init_loop()                                                     //Loops on INIT Press
    {

    }
    @Override
    public void start()                                                         //Runs on start button press
    {
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    //Sets liftDrive to hold at beginning
        runtime.reset();
    }
    @Override
    public void loop()                                                          //Loops While Running
    {
        double vertComp = gamepad1.left_stick_y;
        double horzComp = gamepad1.left_stick_x;
        double spinComp = gamepad1.right_stick_x;

        leftDriveF.setPower (Range.clip((Math.pow((vertComp+horzComp-spinComp),EXPO)),-1,1));
        leftDriveB.setPower (Range.clip((Math.pow((vertComp-horzComp-spinComp),EXPO)),-1,1));
        rightDriveF.setPower(Range.clip((Math.pow((vertComp-horzComp+spinComp),EXPO)),-1,1));
        rightDriveB.setPower(Range.clip((Math.pow((vertComp+horzComp+spinComp),EXPO)),-1,1));

        if (!gamepad1.y)
        {
            clawDrive.setPower(Range.clip((gamepad1.right_trigger-gamepad1.left_trigger),-1,1));
        }
        else
        {
            if (gamepad1.right_trigger>0){liftDrive.setPower(Range.clip((Math.pow((gamepad1.right_trigger),EXPO)),-1,1));}
            else {liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        }
        telemetry.addData("Input LX",horzComp);
        telemetry.addData("Input RX",spinComp);
        telemetry.addData("Input LY",vertComp);
        telemetry.addData("Input TR",gamepad1.right_trigger);
        telemetry.addData("Input TL",gamepad1.left_trigger);
        telemetry.addData ("LF-PWR",Double.toString(leftDriveF.getPower()));
        telemetry.addData ("LB-PWR",Double.toString(leftDriveB.getPower()));
        telemetry.addData("RF-PWR",Double.toString(rightDriveF.getPower()));
        telemetry.addData("RB-PWR",Double.toString(rightDriveB.getPower()));
        telemetry.update();                                                     //Make sure telemetry Updates
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }

}
