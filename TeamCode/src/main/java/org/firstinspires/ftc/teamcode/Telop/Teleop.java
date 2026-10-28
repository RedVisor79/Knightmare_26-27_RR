package org.firstinspires.ftc.teamcode.Telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    // Drivetrain (goBILDA mecanum)
    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C

    // Mechanisms
    private DcMotorEx Nectar; // 0E  flywheel
    private DcMotorEx Pollen; // 1E  flywheel
    private DcMotor LS;       // linear slide
    private DcMotor INTAKE;   // 3E
    private ServoImplEx FlowerGate;
    private ServoImplEx FlywheelGate;

    // Drive power values (for telemetry)
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;

    // State
    boolean nectarOn = false;
    boolean pollenOn = false;
    boolean flowerGateOpen = false;
    boolean flywheelGateOpen = false;
    boolean lastDpadRight = false;
    boolean lastDpadLeft = false;
    String lsStatus = "NOT IN USE";

    // ----- Tunable in FTC Dashboard -----
    public static double NECTAR_VEL = 1750; // ticks/sec (must be below your motor's max)
    public static double POLLEN_VEL = 1750;
    public static double LS_Sens = 0.75;
    public static double INTAKE_POWER = 1.0;
    public static double TRIGGER_DEADZONE = 0.1;
    public static double STRAFE_MULT = 1.0; // try up to ~1.1 if strafing feels weaker than driving

    // Gate angles: how far (in degrees) each gate moves from the 135° center
    // closed = 135 - ANGLE, open = 135 + ANGLE (swap the sign if a gate moves the wrong way)
    public static double FLOWER_ANGLE = 90;
    public static double FLYWHEEL_ANGLE = 90;
    private static final double SERVO_RANGE_DEG = 270.0;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Faster loop times: read all hub sensors/encoders in one bulk read per loop
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Nectar = hardwareMap.get(DcMotorEx.class, "NECTAR");
        Pollen = hardwareMap.get(DcMotorEx.class, "POLLEN");
        LS = hardwareMap.get(DcMotor.class, "LS");
        INTAKE = hardwareMap.get(DcMotor.class, "Intake");
        FlowerGate = hardwareMap.get(ServoImplEx.class, "FlowerGate");
        FlywheelGate = hardwareMap.get(ServoImplEx.class, "FlywheelGate");

        // Directions
        // goBILDA mecanum: motors on one side face the opposite way, so reverse the left side.
        // If the robot drives backwards, reverse the RIGHT side instead (swap these lines).
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        Pollen.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);

        // Drivetrain: encoders on, brake when sticks released
        DcMotor[] driveMotors = {LB, LF, RB, RF};
        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Flywheels: setVelocity() REQUIRES RUN_USING_ENCODER. Coast when off.
        DcMotorEx[] flywheels = {Nectar, Pollen};
        for (DcMotorEx m : flywheels) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Linear slide: hold position when released
        LS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Gates: full 270° range on REV Smart Robot Servos
        FlowerGate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        FlywheelGate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        applyGates(); // start closed (matches flowerGateOpen/flywheelGateOpen = false)

        telemetry.addLine("안녕하세요");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            MecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            Launcher();
            Intake();
            LinearSlide();
            GateControls();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Nectar target/actual", "%.0f / %.0f", nectarOn ? NECTAR_VEL : 0, Nectar.getVelocity());
            telemetry.addData("Pollen target/actual", "%.0f / %.0f", pollenOn ? POLLEN_VEL : 0, Pollen.getVelocity());
            telemetry.addData("Linear Slide", LS_Sens + " " + lsStatus);
            telemetry.addData("FlowerGate", flowerGateOpen ? "OPEN" : "CLOSED");
            telemetry.addData("FlywheelGate", flywheelGateOpen ? "OPEN" : "CLOSED");
            telemetry.addData("LB", lbPower);
            telemetry.addData("LF", lfPower);
            telemetry.addData("RB", rbPower);
            telemetry.addData("RF", rfPower);
            telemetry.update();
        }
    }

    // B = start Nectar, A = start Pollen, X = stop both.
    // Velocity is re-applied every loop so Dashboard changes take effect live.
    private void Launcher() {
        if (gamepad1.b) nectarOn = true;
        if (gamepad1.a) pollenOn = true;
        if (gamepad1.x) {
            nectarOn = false;
            pollenOn = false;
        }

        Nectar.setVelocity(nectarOn ? NECTAR_VEL : 0);
        Pollen.setVelocity(pollenOn ? POLLEN_VEL : 0);
    }

    // Dpad up/down = slide up/down (held)
    private void LinearSlide() {
        if (gamepad1.dpad_up) {
            lsStatus = "UPWARDS";
            LS.setPower(LS_Sens);
        } else if (gamepad1.dpad_down) {
            lsStatus = "DOWNWARDS";
            LS.setPower(-LS_Sens);
        } else {
            lsStatus = "NOT IN USE";
            LS.setPower(0);
        }
    }

    private void Intake() {
        if (gamepad1.left_trigger > TRIGGER_DEADZONE)
            INTAKE.setPower(INTAKE_POWER);
        else if (gamepad1.left_bumper)
            INTAKE.setPower(-INTAKE_POWER);
        else
            INTAKE.setPower(0);
    }

    // Dpad right = toggle FlowerGate, Dpad left = toggle FlywheelGate.
    // Edge detection: one press = one toggle (holding the button does NOT keep flipping).
    private void GateControls() {
        if (gamepad1.dpad_right && !lastDpadRight) flowerGateOpen = !flowerGateOpen;
        if (gamepad1.dpad_left && !lastDpadLeft) flywheelGateOpen = !flywheelGateOpen;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;

        applyGates();
    }

    private void applyGates() {
        FlowerGate.setPosition(gatePosition(flowerGateOpen, FLOWER_ANGLE));
        FlywheelGate.setPosition(gatePosition(flywheelGateOpen, FLYWHEEL_ANGLE));
    }

    private double gatePosition(boolean open, double angle) {
        double deg = open ? 135 + angle : 135 - angle;
        return Range.clip(deg / SERVO_RANGE_DEG, 0.0, 1.0);
    }

    // Hold left trigger = intake, hold left bumper = unjam (reverse)


    private void MecanumDrive(double forward, double strafe, double turn) {
        strafe *= STRAFE_MULT;

        lbPower = forward - strafe + turn;
        lfPower = forward + strafe + turn;
        rbPower = forward + strafe - turn;
        rfPower = forward - strafe - turn;

        // Normalize so no wheel exceeds 1.0 while keeping the ratios
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