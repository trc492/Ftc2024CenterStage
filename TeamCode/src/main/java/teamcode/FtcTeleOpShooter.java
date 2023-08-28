/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

@TeleOp(name="FtcTeleOpShooter", group="Ftc3543")
public class FtcTeleOpShooter extends FtcOpMode
{
    private static final double GOBILDA_5203_GEAR_RATIO = (1.0 + (46.0 / 17.0));
    private static final double GOBILDA_5203_MAX_RPM = 2620.0;
    private static final double GOBILDA_5203_ENCODER_PPR = 28.0 * GOBILDA_5203_GEAR_RATIO;

    private static final double MOTOR_SCALE = 1.0 / GOBILDA_5203_ENCODER_PPR;
    private static final double MOTOR_OFFSET = 0.0;

    private static final double MOTOR_VEL_KP = 10.0;
    private static final double MOTOR_VEL_KI = 3.0;
    private static final double MOTOR_VEL_KD = 0.0;
    private static final double MOTOR_VEL_KF = 0.0;

    private static final double MOTOR_MIN_POWER = -1.0;
    private static final double MOTOR_MAX_POWER = 1.0;
    private static final double MOTOR_POWER_INC = 0.1;
    private static final int MIN_POWER_INC = (int) Math.floor(MOTOR_MIN_POWER / MOTOR_POWER_INC);
    private static final int MAX_POWER_INC = (int) Math.ceil(MOTOR_MAX_POWER / MOTOR_POWER_INC);
    private static final double MOTOR_MIN_RPM = -GOBILDA_5203_MAX_RPM;
    private static final double MOTOR_MAX_RPM = GOBILDA_5203_MAX_RPM;
    private static final double MOTOR_VEL_INC = 100.0;
    private static final int MIN_VEL_INC = (int) Math.floor(MOTOR_MIN_RPM / MOTOR_VEL_INC);
    private static final int MAX_VEL_INC = (int) Math.ceil(MOTOR_MAX_RPM / MOTOR_VEL_INC);

    private FtcDashboard dashboard;
    private FtcGamepad gamepad;
    private FtcDcMotor motor1;
    private FtcDcMotor motor2;
    private boolean velocityMode = true;
    private int incCount = 0;
    private double motorValue = 0.0;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        dashboard = FtcDashboard.getInstance();

        gamepad = new FtcGamepad("Gamepad", gamepad1, this::gamepadButtonEvent);
        gamepad.setYInverted(true);

        motor1 = new FtcDcMotor("motor1");
        motor1.setMotorInverted(true);
        motor1.setPositionSensorScaleAndOffset(MOTOR_SCALE, MOTOR_OFFSET);
//        motor1.setVelocityPidCoefficients(MOTOR_VEL_KP, MOTOR_VEL_KI, MOTOR_VEL_KD, MOTOR_VEL_KF);
        motor2 = new FtcDcMotor("motor2");
        motor2.setPositionSensorScaleAndOffset(MOTOR_SCALE, MOTOR_OFFSET);
//        motor2.setVelocityPidCoefficients(MOTOR_VEL_KP, MOTOR_VEL_KI, MOTOR_VEL_KD, MOTOR_VEL_KF);
//        motor2.followMotor(motor1);
    }   //initRobot

    private void setMotorValue(boolean velMode, double value)
    {
        if (velMode)
        {
            motor1.setVelocity(value);
            motor2.setVelocity(value);
            dashboard.displayPrintf(1, "VelMode: motorValue=%.1f", value);
        }
        else
        {
            motor1.setPower(value);
            motor2.setPower(value);
            dashboard.displayPrintf(1, "PwrMode: motorValue=%.3f", value);
//            dashboard.displayPrintf(6, "ShooterSetMotorValue: power=%.3f", value);
        }
    }   //setMotorValue

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();

        TrcPidController.PidCoefficients pidCoeff1 = motor1.getVelocityPidCoefficients();
        dashboard.displayPrintf(3, "Motor1: PIDCoeff=%s", pidCoeff1);

        TrcPidController.PidCoefficients pidCoeff2 = motor2.getVelocityPidCoefficients();
        dashboard.displayPrintf(4, "Motor2: PIDCoeff=%s", pidCoeff2);
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            double value = gamepad.getRightStickY(false);

            if (value != 0.0)
            {
                // Analog control by right stick.
                if (velocityMode)
                {
                    value *= MOTOR_MAX_RPM;
                }
            }
            else
            {
                // Digital control by DPad.
                value = motorValue;
            }
            setMotorValue(velocityMode, value);
            dashboard.displayPrintf(
                2, "RPM: motor1=%.0f, motor2=%.0f", motor1.getVelocity(), motor2.getVelocity());
        }
    }   //periodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void gamepadButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                if (pressed)
                {
                    velocityMode = !velocityMode;
                    incCount = 0;
                    motorValue = 0.0;
                    setMotorValue(false, 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (pressed)
                {
                    if (velocityMode)
                    {
                        if (incCount < MAX_VEL_INC)
                        {
                            incCount++;
                        }
                        motorValue = incCount * MOTOR_VEL_INC;
                        if (motorValue > MOTOR_MAX_RPM)
                        {
                            motorValue = MOTOR_MAX_RPM;
                        }
                    }
                    else
                    {
                        if (incCount < MAX_POWER_INC)
                        {
                            incCount++;
                        }
                        motorValue = incCount * MOTOR_POWER_INC;
                        if (motorValue > MOTOR_MAX_POWER)
                        {
                            motorValue = MOTOR_MAX_POWER;
                        }
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (pressed)
                {
                    if (velocityMode)
                    {
                        if (incCount > MIN_VEL_INC)
                        {
                            incCount--;
                        }
                        motorValue = incCount * MOTOR_VEL_INC;
                        if (motorValue < MOTOR_MIN_RPM)
                        {
                            motorValue = MOTOR_MIN_RPM;
                        }
                    }
                    else
                    {
                        if (incCount > MIN_POWER_INC)
                        {
                            incCount--;
                        }
                        motorValue = incCount * MOTOR_POWER_INC;
                        if (motorValue < MOTOR_MIN_POWER)
                        {
                            motorValue = MOTOR_MIN_POWER;
                        }
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                break;

            case FtcGamepad.GAMEPAD_BACK:
                break;
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOpShooter
