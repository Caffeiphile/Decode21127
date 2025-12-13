package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants.POD_OFFSET_FROM_CENTER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name = "Complete Robot TeleOp", group = "Drive")
public class CompleteRobotTeleOp extends LinearOpMode {

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;

    private CRServo intake1;
    private CRServo intake2;
    private CRServo intake3;
    private CRServo intake4;

    private static final double FLYWHEEL_TARGET_VELOCITY = 2800;
    private static final double FLYWHEEL_KP = 0.0001;
    private static final double FLYWHEEL_KI = 0.00;
    private static final double FLYWHEEL_KD = 0.0;
    private static final double FLYWHEEL_KF = 0.00012;

    @Override
    public void runOpMode() {

        // === SWERVE SETUP ===
        SimpleSwerveModule frontLeft = createModule(
                "cidTop", "cidBottom", "cidEncoder",
                true, false, -3.0978);
        SimpleSwerveModule backRight = createModule(
                "kraiTop", "kraiBottom", "kraiEncoder",
                true, false, -0.0952);

        SimpleSwerveModule[] modules = {frontLeft, backRight};
        Vector2d[] positions = {
                new Vector2d(POD_OFFSET_FROM_CENTER, POD_OFFSET_FROM_CENTER),
                new Vector2d(-POD_OFFSET_FROM_CENTER, -POD_OFFSET_FROM_CENTER)
        };

        SimpleSwerveDrive drive = new SimpleSwerveDrive(modules, positions);

        // === FLYWHEEL SETUP ===
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        flywheelLeft.setVelocityPIDFCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KF);
        flywheelRight.setVelocityPIDFCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KF);

        // === INTAKE SETUP ===
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        intake3 = hardwareMap.get(CRServo.class, "intake3");
        intake4 = hardwareMap.get(CRServo.class, "intake4");

        waitForStart();

        while (opModeIsActive()) {

            // === SWERVE DRIVE ===
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            drive.drive(forward, strafe, rotate);

            flywheelLeft.setPower(1);
            flywheelRight.setPower(1);
            // === FLYWHEEL CONTROL ===
            if (gamepad1.right_bumper) {
                flywheelLeft.setVelocity(FLYWHEEL_TARGET_VELOCITY);
                flywheelRight.setVelocity(FLYWHEEL_TARGET_VELOCITY);
                flywheelLeft.setPower(1);
                flywheelRight.setPower(1);
            } else {
                flywheelLeft.setPower(0);
                flywheelRight.setPower(0);
            }

            // === INTAKE CONTROL ===
            double intakePower = 0;
            if (gamepad1.left_bumper) {
                intakePower = -1;
            } else if (gamepad1.left_trigger > 0.1) {
                intakePower = 1;
            }

            intake1.setPower(intakePower);
            intake2.setPower(-intakePower);
            intake3.setPower(intakePower);
            intake4.setPower(-intakePower);

            // === TELEMETRY ===
            telemetry.addData("Drive", "F: %.2f S: %.2f R: %.2f", forward, strafe, rotate);
            telemetry.addData("Flywheel L", "%.0f RPM", flywheelLeft.getVelocity());
            telemetry.addData("Flywheel R", "%.0f RPM", flywheelRight.getVelocity());
            telemetry.addData("Intake", intakePower > 0 ? "IN" : (intakePower < 0 ? "OUT" : "OFF"));
            telemetry.update();
        }
    }

    private SimpleSwerveModule createModule(
            String top, String bottom, String encoder,
            boolean topRev, boolean bottomRev, double offset) {

        SwerveModuleHardware hw = new SwerveModuleHardware(
                hardwareMap, top, bottom, encoder,
                topRev, bottomRev, offset);

        SteeringPIDController pid = new SteeringPIDController(0.80, 0.012, 0.0);

        return new SimpleSwerveModule(hw, pid);
    }
}