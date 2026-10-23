package org.firstinspires.ftc.teamcode.Telop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C

    // Drive power values
    private DcMotorEx Nectar; // 0E
    private DcMotorEx Pollen; // 1E
    private DcMotor LS;
    private DcMotor INTAKE; // 3E
    private Servo Gate1;
    private Servo Gate2;

    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    boolean shootingNectar = false;
    boolean shootingPollen = false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double NECTAR_VEL = 1750;
    public static double POLLEN_VEL = 1750;
    public static double LS_Sens = 0.75;
    public static double ANGLE1 = 90;
    public static boolean OPEN1 = false;
    public static double ANGLE2 = 90;
    public static boolean OPEN2 = false;
    public static String  LS_Direction = "NOT IN USE";

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Nectar = hardwareMap.get(DcMotorEx.class, "NECTAR");
        Pollen = hardwareMap.get(DcMotorEx.class, "POLLEN");
        LS = hardwareMap.get(DcMotor.class, "LS");
        INTAKE = hardwareMap.get(DcMotor.class, "Intake");
        Gate1 = hardwareMap.get(Servo.class, "Gate1");
        Gate2 = hardwareMap.get(Servo.class, "Gate2");


        // Directions
        //LB.setDirection(DcMotor.Direction.REVERSE);
        //LF.setDirection(DcMotor.Direction.REVERSE);
        //RB.setDirection(DcMotor.Direction.REVERSE);
        //RF.setDirection(DcMotor.Direction.REVERSE);
        //Nectar.setDirection(DcMotor.Direction.REVERSE);
        Pollen.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);
        //INTAKE.setDirection(DcMotor.Direction.REVERSE);
        Gate1.setPosition(0.0);
        Gate2.setPosition(0.0);

        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //telemetry.addLine("Здравствуйте!");
        telemetry.addLine("안녕하세요");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            MecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x );
            Launcher();
            Intake();
            LinearSlide();
            GateControls();


            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Nectar:", NECTAR_VEL);
            telemetry.addData("Pollen:", POLLEN_VEL);
            telemetry.addData("Linear Slide:", LS_Sens+" "+LS_Direction);
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.update();
        }
    }

    // Shooter control
    private void Launcher() {

        if (gamepad1.b){
            Nectar.setVelocity(NECTAR_VEL);
        }
        if (gamepad1.a){
            Pollen.setVelocity(POLLEN_VEL);
        }
        if (gamepad1.x){
            Nectar.setVelocity(0);
            Pollen.setVelocity(0);
            shootingNectar = false;
            shootingPollen = false;
        }
        if (gamepad1.right_bumper&&!shootingNectar&&!shootingPollen){
            Nectar.setVelocity(-NECTAR_VEL);
            Pollen.setVelocity(-POLLEN_VEL);
            shootingNectar=true;
            shootingPollen=true;
            sleep(10);
            Nectar.setVelocity(0);
            Pollen.setVelocity(0);
            shootingNectar = false;
            shootingPollen = false;
        }
    }
    /*private void PollenLauncher() {

        if (gamepad1.a){
            Pollen.setVelocity(POLLEN_VEL);
        }
        if (gamepad1.x){
            Pollen.setVelocity(0);
            shootingPollen = false;
        }
        if (gamepad1.y&&!shootingPollen){
            Pollen.setVelocity(-POLLEN_VEL);
            shootingPollen=true;
            sleep(10);
            Pollen.setVelocity(0);
            shootingPollen = false;
        }
    }*/

    private void LinearSlide() {
        if (gamepad1.dpad_up){
            LS_Direction = "UPWARDS";
            LS.setPower(LS_Sens);
        }
        else if (gamepad1.dpad_down){
            LS_Direction = "DOWNWARDS";
            LS.setPower(-LS_Sens);
        }
        else {
            LS_Direction = "NOT IN USE";
            LS.setPower(0);
        }

    }

    public void GateControls(){
        if (gamepad1.dpad_right){
            if (OPEN1) Gate1.setPosition((135+ANGLE1)/270);
            else Gate1.setPosition((135-ANGLE1)/270);
            OPEN1= !OPEN1;
        }
        if (gamepad1.dpad_left){
            if (OPEN2) Gate2.setPosition((135+ANGLE2)/270);
            else Gate2.setPosition((135-ANGLE2)/270);
            OPEN2=!OPEN2;
        }
    }


    private void Intake() {
        if (gamepad1.left_trigger>0)
            INTAKE.setPower(1);
        else if (gamepad1.left_bumper)
            INTAKE.setPower(-1);
        else
            INTAKE.setPower(0);
        }

    private void MecanumDrive(double X, double Y, double R){
        // Mecanum drive calculations

        double forward = -X;
        double strafe = Y;
        double turn = R;

        lbPower = forward - strafe + turn;
        lfPower = forward + strafe + turn;
        rbPower = forward + strafe - turn;
        rfPower = forward - strafe - turn;

        // Normalize
        double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower)));

        if (max > 1.0) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        LB.setPower(lbPower);
        LF.setPower(lfPower);
        RB.setPower(rbPower);
        RF.setPower(rfPower);
    }
}
