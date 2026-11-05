package org.firstinspires.ftc.teamcode.Teleop;

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
@TeleOp(name = "AutoAimAndShoot")
public class AutoAimAndShoot extends LinearOpMode {

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

    // PID-controlled linear slide
    private LinearSlideController slide;

    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    boolean shootingNectar = false;
    boolean shootingPollen = false;

    // Base dashboard variables
    public static double BASE_NECTAR_VEL = 17500;
    public static double BASE_POLLEN_VEL = 17500;
    private static boolean OPEN1 = false;
    private static boolean OPEN2 = false;

    // --- DUAL AIMING SYSTEM CONSTANTS ---
    private final int CENTER_X_PIXEL = 160;  
    private final int PIXEL_THRESHOLD = 5;   
    private final double AUTO_TURN_SPEED = 0.15; 

    // Nectar Target Configuration (e.g., Hives)
    private final int NECTAR_TAG_ID = 1;     
    private final int NECTAR_CLOSE_HEIGHT = 100; 
    private final int NECTAR_FAR_HEIGHT = 30;    
    private final double NECTAR_MIN_VEL = 14000; 
    private final double NECTAR_MAX_VEL = 24000; 

    // Pollen Target Configuration (e.g., Flowers)
    private final int POLLEN_TAG_ID = 2;     
    private final int POLLEN_CLOSE_HEIGHT = 90;  
    private final int POLLEN_FAR_HEIGHT = 25;    
    private final double POLLEN_MIN_VEL = 11000; 
    private final double POLLEN_MAX_VEL = 19000; 

    // Telemetry tracking variables
    private double dynamicNectarVel = 17500;
    private double dynamicPollenVel = 17500;
    private int currentTagHeight = 0;
    private String activeAimSystem = "Manual";

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

        // Directions & Defaults
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        Pollen.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);
        Gate1.setPosition(0.0);
        Gate2.setPosition(0.0);

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!huskylens.initialize()) {
            telemetry.addData("Error", "HuskyLens failed to initialize!");
        }
        huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        slide = new LinearSlideController(LS);
        slide.init();

        telemetry.addLine("BioBuzz Dual-Aim Initialized");
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

            telemetry.addData("Active Aim System", activeAimSystem);
            telemetry.addData("Tag Height", currentTagHeight);
            telemetry.addData("Dynamic Nectar Vel", dynamicNectarVel);
            telemetry.addData("Dynamic Pollen Vel", dynamicPollenVel);
            telemetry.addData("Slide Pos", slide.getPosition());
            telemetry.update();
        }
    }

    private void Launcher() {
        if (gamepad1.y) {
            Nectar.setVelocity(dynamicNectarVel);
        } else if (gamepad1.x) {
            Pollen.setVelocity(dynamicPollenVel);
        } else if (gamepad1.b){
            Nectar.setVelocity(BASE_NECTAR_VEL);
        } else if (gamepad1.a){
            Pollen.setVelocity(BASE_POLLEN_VEL);
        } else {
            Nectar.setVelocity(0);
            Pollen.setVelocity(0);
        }
    }

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

        // Call the separate auto-aim function depending on which button is held y=nectar x=pollen
        if (gamepad1.y) {
            activeAimSystem = "Nectar Aim (Tag ID " + NECTAR_TAG_ID + ")";
            turn = autoAimAndShoot(NECTAR_TAG_ID, NECTAR_CLOSE_HEIGHT, NECTAR_FAR_HEIGHT, NECTAR_MIN_VEL, NECTAR_MAX_VEL, true);
        } else if (gamepad1.x) {
            activeAimSystem = "Pollen Aim (Tag ID " + POLLEN_TAG_ID + ")";
            turn = autoAimAndShoot(POLLEN_TAG_ID, POLLEN_CLOSE_HEIGHT, POLLEN_FAR_HEIGHT, POLLEN_MIN_VEL, POLLEN_MAX_VEL, false);
        } else {
            activeAimSystem = "Manual Driver Control";
        }

        MecanumDrive(forward, strafe, turn);
    }

   
    private double autoAimAndShoot(int targetTagId, int closeHeight, int farHeight, double minVel, double maxVel, boolean isNectar) {
        HuskyLens.Block[] blocks = huskylens.blocks();
        boolean targetFound = false;
        int targetX = 0;
        int targetHeight = 0;

        for (HuskyLens.Block block : blocks) {
            if (block.id == targetTagId) {
                targetFound = true;
                targetX = block.x;
                targetHeight = block.height;
                break;
            }
        }

        double calculatedTurn = AUTO_TURN_SPEED;

        if (targetFound) {
            currentTagHeight = targetHeight;

            // Horizontal Steering Alignment Error Check
            int error = targetX - CENTER_X_PIXEL;
            if (Math.abs(error) > PIXEL_THRESHOLD) {
                calculatedTurn = (error > 0) ? AUTO_TURN_SPEED : -AUTO_TURN_SPEED;
            } else {
                calculatedTurn = 0; 
            }

            // Element-Specific Distance RPM Calculation
            double distanceRatio = (double)(targetHeight - farHeight) / (closeHeight - farHeight);
            distanceRatio = Range.clip(distanceRatio, 0.0, 1.0);

            double targetVel = maxVel - (distanceRatio * (maxVel - minVel));
            if (isNectar) {
                dynamicNectarVel = targetVel;
            } else {
                dynamicPollenVel = targetVel;
            }
        } else {
            // Automatic search spin if target tag is out of view
            calculatedTurn = AUTO_TURN_SPEED;
        }

        return calculatedTurn;
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
