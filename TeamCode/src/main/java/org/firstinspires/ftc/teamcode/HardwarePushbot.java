package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwarePushbot {
    static final double COUNTS_PER_INCH = Constants.COUNTS_PER_MOTOR_REV / (Constants.WHEEL_DIAMATER_INCHES * Math.PI);

    /* Public OpMode members */
    public DcMotor frontL;
    public DcMotor frontR;
    public DcMotor backL;
    public DcMotor backR;

    public BNO055IMU imu;

    /* local OpMode members */
    HardwareMap hwMap;

    public void setMotorPowers(double power) {
        frontL.setPower(power);
        frontR.setPower(power);
        backL.setPower(power);
        backR.setPower(power);
    }

    public void setMotorPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        frontL.setPower(fLPower);
        frontR.setPower(fRPower);
        backL.setPower(bLPower);
        backR.setPower(bRPower);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        frontL.setMode(mode);
        frontR.setMode(mode);
        backL.setMode(mode);
        backR.setMode(mode);
    }

    public void encoderDrive(LinearOpMode op, double speed,
                             double frontLeft, double frontRight, double backLeft, double backRight) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (op.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLFTarget = frontL.getCurrentPosition() + (int)(frontLeft * COUNTS_PER_INCH);
            newRFTarget = frontR.getCurrentPosition() + (int)(frontRight * COUNTS_PER_INCH);
            newLBTarget = backL.getCurrentPosition() + (int)(backLeft * COUNTS_PER_INCH);
            newRBTarget = backR.getCurrentPosition() + (int)(backRight * COUNTS_PER_INCH);

            frontL.setTargetPosition(newLFTarget);
            frontR.setTargetPosition(newRFTarget);
            backL.setTargetPosition(newLBTarget);
            backR.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion
            frontL.setPower(Math.abs(speed));
            frontR.setPower(Math.abs(speed));
            backL.setPower(Math.abs(speed));
            backR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (op.opModeIsActive() &&
                    (frontL.isBusy() && frontR.isBusy()
                            && backL.isBusy() && backR.isBusy())) {
            }

            // Stop all motion;
            setMotorPowers(0);

            // Turn off RUN_TO_POSITION
            setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontL = hwMap.get(DcMotor.class, "frontLeft");
        frontR = hwMap.get(DcMotor.class, "frontRight");
        backL = hwMap.get(DcMotor.class, "backLeft");
        backR = hwMap.get(DcMotor.class, "backRight");

        frontL.setDirection(DcMotor.Direction.FORWARD);
        frontR.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);

        setMotorPowers(0); // Set all motors to zero power on initialization

        // Gyro Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize gyro
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
