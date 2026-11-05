package org.firstinspires.ftc.teamcode.Telop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private HuskyLens huskylens;

    // PID-controlled linear slide (see LinearSlideController.java)
    private LinearSlideController slide;

    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    boolean shootingNectar = false;
    boolean shootingPollen = false;

    // Shooter velocity (tunable base values in FTC Dashboard)
    public static double BASE_NECTAR_VEL = 17500;
    public static double BASE_POLLEN_VEL = 17500;
    private static boolean OPEN1 = false;
    private static boolean OPEN2 = false;

    // HuskyLens Aim-Assist & Dynamic Distance Tuning Constants
    private final int TARGET_TAG_ID = 1;     // Change to your desired AprilTag ID
    private final int CENTER_X_PIXEL = 160;  // Center X frame coordinate (320 / 2)
    private final int PIXEL_THRESHOLD = 5;   // Error margin for centering
    private final double AUTO_TURN_SPEED = 0.15; // Auto-search & align rotation speed

    // Distance/RPM scaling constants (tune based on your robot's physical testing)
    private final int CLOSE_TAG_HEIGHT = 100; // Pixel height of tag when robot is close to goal
    private final int FAR_TAG_HEIGHT = 30;    // Pixel height of tag when robot is far from goal
    private final double MIN_VELOCITY = 12000; // Minimum shooter RPM for close shots
    private final double MAX_VELOCITY = 22000; // Maximum shooter RPM for far shots

    // Live variables to track dynamic calculations for telemetry
    private double dynamicNectarVel = 17500;
    private double dynamicPollenVel = 17500;
    private int currentTagHeight = 0;

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
        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");

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
            telemetry.addData("Tag Height (Pixels)", currentTagHeight);
            telemetry.addData("Dynamic Nectar Vel", dynamicNectarVel);
            telemetry.addData("Dynamic Pollen Vel", dynamicPollenVel);
            telemetry.addData("Slide Target", slide.getTarget());
            telemetry.addData("Slide Pos", slide.getPosition());
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.update();
        }
    }

    // Shooter control
    private void Launcher() {
        // If holding Y (auto-aim), we can optionally apply the dynamically calculated velocities.
        // Otherwise, manual buttons use the static dashboard variables.
        if (gamepad1.y) {
            Nectar.setVelocity(dynamicNectarVel);
            Pollen.setVelocity(dynamicPollenVel);
        } else if (gamepad1.b){
            Nectar.setVelocity(BASE_NECTAR_VEL);
        } else if (gamepad1.a){
            Pollen.setVelocity(BASE_POLLEN_VEL);
        } else if (gamepad1.x){
            Nectar.setVelocity(0);
            Pollen.setVelocity(0);
            shootingNectar = false;
            shootingPollen = false;
        }
        
        if (gamepad1.right_bumper && !shootingNectar && !shootingPollen){
            Nectar.setVelocity(-BASE_NECTAR_VEL);
            Pollen.setVelocity(-BASE_POLLEN_VEL);
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

        // --- HuskyLens Auto-Aim, Search Assist, & Distance RPM Scaling ---
        if (gamepad1.y) {
            HuskyLens.Block[] blocks = huskylens.blocks();
            boolean targetFound = false;
            int targetX = 0;
            int targetHeight = 0;

            for (HuskyLens.Block block : blocks) {
                if (block.id == TARGET_TAG_ID) {
                    targetFound = true;
                    targetX = block.x;
                    targetHeight = block.height; // Capture height of AprilTag bounding box
                    break;
                }
            }

            if (targetFound) {
                currentTagHeight = targetHeight;

                // 1. Horizontal Steering Alignment
                int error = targetX - CENTER_X_PIXEL;
                if (Math.abs(error) > PIXEL_THRESHOLD) {
                    turn = (error > 0) ? AUTO_TURN_SPEED : -AUTO_TURN_SPEED;
                } else {
                    turn = 0; 
                    telemetry.addData("Aim Assist", "LOCKED ON TARGET!");
                }

                // 2. Dynamic Distance-Based RPM Calculation
                // As tag height increases (robot is closer), velocity decreases. 
                // As tag height decreases (robot is farther), velocity increases.
                double distanceRatio = (double)(targetHeight - FAR_TAG_HEIGHT) / (CLOSE_TAG_HEIGHT - FAR_TAG_HEIGHT);
                distanceRatio = Range.clip(distanceRatio, 0.0, 1.0); // Keep between 0 and 1

                // Interpolate target velocity between MIN and MAX velocities based on distance
                dynamicNectarVel = MAX_VELOCITY - (distanceRatio * (MAX_VELOCITY - MIN_VELOCITY));
                dynamicPollenVel = MAX_VELOCITY - (distanceRatio * (MAX_VELOCITY - MIN_VELOCITY));

            } else {
                // Search pattern if tag is missing
                turn = AUTO_TURN_SPEED;
                telemetry.addData("Aim Assist", "Searching for Tag ID %d...", TARGET_TAG_ID);
            }
        }

        MecanumDrive(forward, strafe, turn);
    }

    private void MecanumDrive(double forward, double strafe, double turn){
        lbPower = forward - strafe + turn;
        lfPower = forward + strafe + turn;
        rbPower = forward + strafe - turn;
        rfPower = forward - strafe - turn;

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
