//This code was created to be used with the HuskLens AI Camera if you plan on using limelight please let me know

package org.firstinspires.ftc.teamcode.Telop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    // Drivetrain Motors
    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C

    // Mechanisms & Vision
    private DcMotorEx Nectar; // 0E
    private DcMotorEx Pollen; // 1E
    private DcMotor LS;
    private DcMotor INTAKE; // 3E
    private Servo Gate1;
    private Servo Gate2;
    private HuskyLens huskylens; // Added HuskyLens vision sensor

    // PID-controlled linear slide (see LinearSlideController.java)
    private LinearSlideController slide;

    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    boolean shootingNectar = false;
    boolean shootingPollen = false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double NECTAR_VEL = 17500;
    public static double POLLEN_VEL = 17500;
    private static boolean OPEN1 = false;
    private static boolean OPEN2 = false;

    // HuskyLens Aim-Assist Tuning Constants
    private final int TARGET_TAG_ID = 1;     // Change to your desired AprilTag ID
    private final int CENTER_X_PIXEL = 160;  // Center X frame coordinate (320 / 2)
    private final int PIXEL_THRESHOLD = 5;   // Error margin for centering
    private final double AUTO_TURN_SPEED = 0.15; // Auto-search & align rotation speed

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
        huskylens = hardwareMap.get(HuskyLens.class, "huskylens"); // Map HuskyLens

        // Directions
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        Pollen.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);
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

        // Initialize HuskyLens algorithm for AprilTags
        if (!huskylens.initialize()) {
            telemetry.addData("Error", "HuskyLens failed to initialize!");
        }
        huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Linear slide (slide must be fully down at init)
        slide = new LinearSlideController(LS);
        slide.init();

        telemetry.addLine("안녕하세요");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        slide.start();

        while (opModeIsActive()) {
            Drive();
            Launcher();
            Intake();
            LinearSlide();
            FlywheelGate();
            FlowerGate();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Nectar Target:", NECTAR_VEL);
            telemetry.addData("Nectar Actual: ", Nectar.getVelocity());
            telemetry.addData("Pollen:", POLLEN_VEL);
            telemetry.addData("Pollen Actual: ", Pollen.getVelocity());
            telemetry.addData("Slide Target", slide.getTarget());
            telemetry.addData("Slide Pos", slide.getPosition());
            telemetry.addData("Slide Power", slide.getPower());
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.addData("FlowerL ", OPEN1);
            telemetry.addData("FlywheelGate: ", OPEN2);
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
        if (gamepad1.right_bumper && !shootingNectar && !shootingPollen){
            Nectar.setVelocity(-NECTAR_VEL);
            Pollen.setVelocity(-POLLEN_VEL);
            shootingNectar = true;
            shootingPollen = true;
            sleep(10);
            Nectar.setVelocity(0);
            Pollen.setVelocity(0);
            shootingNectar = false;
            shootingPollen = false;
        }
    }

    // Linear slide: dpad_up toggles between up/down positions, held by PID
    private void LinearSlide() {
        slide.update(gamepad1.dpad_up);
    }

    public void FlowerGate(){
        if (gamepad1.dpad_right){
            if (!OPEN1) Gate1.setPosition(1);
            else Gate1.setPosition(0);
            OPEN1 = !OPEN1;
        }
    }

    public void FlywheelGate(){
        if (gamepad1.dpad_left){
            if (!OPEN2) Gate2.setPosition(1);
            else Gate2.setPosition(0);
            OPEN2 = !OPEN2;
        }
    }

    private void Intake() {
        if (gamepad1.left_trigger > 0)
            INTAKE.setPower(1);
        else if (gamepad1.left_bumper)
            INTAKE.setPower(-1);
        else
            INTAKE.setPower(0);
    }

    private void Drive(){
        double forward = gamepad1.right_trigger > 0 ? gamepad1.left_stick_y : -gamepad1.left_stick_y;
        double strafe  = gamepad1.right_trigger > 0 ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
        double turn    = gamepad1.right_stick_x;

        // --- HuskyLens Auto-Aim & Search Assist Override ---
        // Hold Gamepad 1 'Y' to enable automatic tracking and searching.
        if (gamepad1.y) {
            HuskyLens.Block[] blocks = huskylens.blocks();
            boolean targetFound = false;
            int targetX = 0;

            for (HuskyLens.Block block : blocks) {
                if (block.id == TARGET_TAG_ID) {
                    targetFound = true;
                    targetX = block.x;
                    break;
                }
            }

            if (targetFound) {
                int error = targetX - CENTER_X_PIXEL;
                if (Math.abs(error) > PIXEL_THRESHOLD) {
                    turn = (error > 0) ? AUTO_TURN_SPEED : -AUTO_TURN_SPEED;
                } else {
                    turn = 0; // Perfectly centered on the AprilTag goal
                    telemetry.addData("Aim Assist", "LOCKED ON TARGET!");
                }
            } else {
                // If tag is out of view, automatically spin in place to scan for it
                turn = AUTO_TURN_SPEED;
                telemetry.addData("Aim Assist", "Searching for Tag ID %d...", TARGET_TAG_ID);
            }
        }

        MecanumDrive(forward, strafe, turn);
    }

    private void MecanumDrive(double forward, double strafe, double turn){
        // Mecanum drive calculations
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
