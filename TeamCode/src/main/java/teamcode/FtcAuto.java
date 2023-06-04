/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdTimedDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcChoiceMenu;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcMenu;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcValueMenu;
import teamcode.autocommands.CmdAutoHigh;
import teamcode.vision.EocvVision;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="Ftc3543")
public class FtcAuto extends FtcOpMode
{
    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum StartPos
    {
        LEFT,
        RIGHT
    }   //enum StartPos

    public enum AutoStrategy
    {
        CYCLE_HIGH,
        CYCLE_HIGH_PRELOAD_ONLY,
        PARKING_ONLY,
        PID_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum Parking
    {
        NEAR_TILE,
        FAR_TILE,
        NO_PARKING
    }   //enum Parking

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        public double startDelay = 0.0;
        public Alliance alliance = Alliance.RED_ALLIANCE;
        public StartPos startPos = StartPos.LEFT;
        public AutoStrategy strategy = AutoStrategy.DO_NOTHING;
        public Parking parking = Parking.NEAR_TILE;
        public double xTarget = 0.0;
        public double yTarget = 0.0;
        public double turnTarget = 0.0;
        public double driveTime = 0.0;
        public double drivePower = 0.0;
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "startDelay=%.0f " +
                "alliance=\"%s\" " +
                "startPos=\"%s\" " +
                "strategy=\"%s\" " +
                "parking=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%.0f " +
                "driveTime=%.0f " +
                "drivePower=%.1f",
                startDelay, alliance, startPos, strategy, parking,
                xTarget, yTarget, turnTarget, driveTime, drivePower);
        }   //toString

    }   //class AutoChoices

    private static final String moduleName = "FtcAuto";
    public static final AutoChoices autoChoices = new AutoChoices();
    private Robot robot;
    private TrcRobot.RobotCommand autoCommand;

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
        final String funcName = "initRobot";
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            Robot.matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(
                Locale.US, "%s%02d_Auto", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber);
            robot.globalTracer.openTraceLog(RobotParams.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and run choice menus.
        //
        doAutoChoicesMenus();
        //
        // Create autonomous command according to chosen strategy.
        //
        switch (autoChoices.strategy)
        {
            //same constructor for cycle high, preload only, and parking only 
            case CYCLE_HIGH:
            case CYCLE_HIGH_PRELOAD_ONLY:
            case PARKING_ONLY:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoHigh(robot, autoChoices);
                }
                break;

            case PID_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, autoChoices.startDelay,
                        autoChoices.drivePower, null,
                        new TrcPose2D(autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget));
                }
                break;

            case TIMED_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.startDelay, autoChoices.driveTime,
                        0.0, autoChoices.drivePower, 0.0);
                }
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }

        if (robot.vision != null)
        {
            // Enabling vision early so we can detect signal position before match starts.
            if (robot.vision.eocvVision != null)
            {
                robot.globalTracer.traceInfo(funcName, "Enabling EocvVision to detect AprilTag.");
                robot.vision.eocvVision.setDetectObjectType(EocvVision.ObjectType.APRIL_TAG);
            }
            else if (robot.vision.tensorFlowVision != null)
            {
                robot.globalTracer.traceInfo(funcName, "Enabling TensorFlowVision to detect Signal.");
                robot.vision.tensorFlowVision.setEnabled(true);
            }
        }

        robot.zeroCalibrate();

        if (robot.grabber != null)
        {
            robot.setGrabberAutoAssistOn(true);
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically after initRobot() is called but before competition starts. For this season,
     * we are detecting the duck's barcode position before the match starts.
     */
    @Override
    public void initPeriodic()
    {
        // Use vision to detect objects before the match starts.
        if (robot.vision != null && (robot.vision.tensorFlowVision != null || robot.vision.eocvVision != null))
        {
            robot.vision.getDetectedSignal();
        }
    }   //initPeriodic

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
        final String funcName = "startMode";

        if (robot.globalTracer.isTraceLogOpened())
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting autonomous *****");
        if (Robot.matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", "%s", Robot.matchInfo);
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", "%s", autoChoices);
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);

        if (robot.vision != null)
        {
            // We are done with detecting signal with TensorFlow, shut it down.
            if (robot.vision.tensorFlowVision != null)
            {
                robot.globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
                robot.vision.tensorFlowShutdown();
            }
        }

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }
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
        //
        // Opmode is about to stop, cancel autonomous command in progress if any.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics(robot.globalTracer);
        robot.globalTracer.traceInfo(moduleName, "***** Stopping autonomous *****");

        if (robot.globalTracer.isTraceLogOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
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
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //periodic

    /**
     * This method creates the autonomous menus, displays them and stores the choices.
     */
    private void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //
        FtcValueMenu startDelayMenu = new FtcValueMenu(
            "Start delay time:", null, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", startDelayMenu);
        FtcChoiceMenu<StartPos> startPosMenu = new FtcChoiceMenu<>("Start Position:", allianceMenu);
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", startPosMenu);
        FtcChoiceMenu<Parking> parkingMenu = new FtcChoiceMenu<>("Parking:", strategyMenu);

        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0, " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");

        startDelayMenu.setChildMenu(allianceMenu);
        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, startPosMenu);

        startPosMenu.addChoice("Start Position Left", StartPos.LEFT, true, strategyMenu);
        startPosMenu.addChoice("Start Position Right", StartPos.RIGHT, false, strategyMenu);

        strategyMenu.addChoice("Cycle High", AutoStrategy.CYCLE_HIGH, true, parkingMenu);
        strategyMenu.addChoice("Cycle High Preload Only", AutoStrategy.CYCLE_HIGH_PRELOAD_ONLY, false, parkingMenu);
        strategyMenu.addChoice("Park Only", AutoStrategy.PARKING_ONLY, false, parkingMenu);
        strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE, false, xTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        parkingMenu.addChoice("Parking Near Tile", Parking.NEAR_TILE, false);
        parkingMenu.addChoice("Parking Far Tile", Parking.FAR_TILE, true);
        parkingMenu.addChoice("No Parking", Parking.NO_PARKING, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(startDelayMenu);
        //
        // Fetch choices.
        //
        autoChoices.startDelay = startDelayMenu.getCurrentValue();
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.startPos = startPosMenu.getCurrentChoiceObject();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.parking = parkingMenu.getCurrentChoiceObject();
        autoChoices.xTarget = xTargetMenu.getCurrentValue();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.turnTarget = turnTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(2, "Auto Choices: %s", autoChoices);
    }   //doAutoChoicesMenus

}   //class FtcAuto
