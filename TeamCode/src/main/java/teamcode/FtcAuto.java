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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

import ftclib.input.FtcChoiceMenu;
import ftclib.input.FtcMatchInfo;
import ftclib.input.FtcMenu;
import ftclib.robotcore.FtcOpMode;
import ftclib.input.FtcValueMenu;
import teamcode.autocommands.CmdAuto;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="Ftc3543")
public class FtcAuto extends FtcOpMode
{
    private static final String moduleName = FtcAuto.class.getSimpleName();

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum StartPos
    {
        AUDIENCE,
        BACKSTAGE
    }   //enum StartPos

    public enum AutoStrategy
    {
        AUTO_SCORE_2,
        AUTO_SCORE_2PLUS1,
        PID_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum ParkPos
    {
        CORNER,
        CENTER
    }   //enum ParkPos

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        public double delay = 0.0;
        public Alliance alliance = Alliance.RED_ALLIANCE;
        public StartPos startPos = StartPos.BACKSTAGE;
        public AutoStrategy strategy = AutoStrategy.DO_NOTHING;
        public boolean useAprilTagVision = true;
        public ParkPos parkPos = ParkPos.CORNER;
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
                "delay=%.0f " +
                "alliance=\"%s\" " +
                "startPos=\"%s\" " +
                "strategy=\"%s\" " +
                "useAprilTagVision=\"%s\" " +
                "parkPos=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%.0f " +
                "driveTime=%.0f " +
                "drivePower=%.1f",
                delay, alliance, startPos, strategy, useAprilTagVision, parkPos, xTarget, yTarget, turnTarget,
                driveTime, drivePower);
        }   //toString

    }   //class AutoChoices

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
    public void robotInit()
    {
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
            TrcDbgTrace.openTraceLog(RobotParams.LOG_FOLDER_PATH, filePrefix);
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
            case AUTO_SCORE_2PLUS1:
                autoChoices.startPos = StartPos.AUDIENCE;
                //
                // Intentionally fall through to the next state.
                //
            case AUTO_SCORE_2:
                if (RobotParams.Preferences.robotType != RobotParams.RobotType.NoRobot)
                {
                    autoCommand = new CmdAuto(robot, autoChoices);
                }
                break;

            case PID_DRIVE:
                if (RobotParams.Preferences.robotType != RobotParams.RobotType.NoRobot)
                {
                    autoCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, autoChoices.delay,
                        autoChoices.drivePower, null,
                        new TrcPose2D(autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget));
                }
                break;

            case TIMED_DRIVE:
                if (RobotParams.Preferences.robotType != RobotParams.RobotType.NoRobot)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.delay, autoChoices.driveTime,
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
            // Enabling vision early so we can detect team prop position before match starts.
            robot.vision.setActiveWebcam(robot.vision.getFrontWebcam());
            if (autoChoices.alliance == Alliance.RED_ALLIANCE)
            {
                if (robot.vision.redBlobVision != null)
                {
                    robot.globalTracer.traceInfo(moduleName, "Enabling RedBlobVision.");
                    robot.vision.setRedBlobVisionEnabled(true);
                }
            }
            else
            {
                if (robot.vision.blueBlobVision != null)
                {
                    robot.globalTracer.traceInfo(moduleName, "Enabling BlueBlobVision.");
                    robot.vision.setBlueBlobVisionEnabled(true);
                }
            }
        }

        robot.zeroCalibrate();
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically after robotInit() is called but before competition starts. For this season,
     * we are detecting the team prop's position before the match starts.
     */
    @Override
    public void initPeriodic()
    {
        if (robot.vision != null)
        {
            robot.vision.getDetectedTeamPropPosition(autoChoices.alliance, -1);
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
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting autonomous: " + TrcTimer.getCurrentTimeString() + " *****");
        if (Robot.matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", Robot.matchInfo.toString());
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);

        if (robot.vision != null)
        {
            // We are done with detecting TeamProp with TensorFlow, shut it down.
            if (robot.vision.tensorFlowVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Disabling TensorFlowVision.");
                robot.vision.setTensorFlowVisionEnabled(false);
            }
            // We are done with detecting TeamProp with ColorBlob detection, shut it down.
            robot.vision.setRedBlobVisionEnabled(false);
            robot.vision.setBlueBlobVisionEnabled(false);
            // Turn on AprilTag detection with rear camera.
            robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
            robot.vision.setActiveWebcam(robot.vision.getRearWebcam());
            robot.vision.setAprilTagVisionEnabled(true);
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

        printPerformanceMetrics();
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping autonomous: " + TrcTimer.getCurrentTimeString() + " *****");

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
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
        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", null, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", delayMenu);
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", allianceMenu);
        FtcChoiceMenu<StartPos> startPosMenu = new FtcChoiceMenu<>("Start Position:", strategyMenu);
        FtcChoiceMenu<Boolean> useAprilTagVisionMenu = new FtcChoiceMenu<>("AprilTag Vision:", strategyMenu);
        FtcChoiceMenu<ParkPos> parkPosMenu = new FtcChoiceMenu<>("Park Position:", useAprilTagVisionMenu);

        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0, " %.0f deg");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
        // Link Value Menus to their children.
        delayMenu.setChildMenu(allianceMenu);
        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, strategyMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, strategyMenu);

        strategyMenu.addChoice("Autonomous Score 2", AutoStrategy.AUTO_SCORE_2, true, startPosMenu);
        strategyMenu.addChoice("Autonomous Score 2+1", AutoStrategy.AUTO_SCORE_2PLUS1, false, useAprilTagVisionMenu);
        strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE, false, xTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        startPosMenu.addChoice("Start Position Audience", StartPos.AUDIENCE, true, useAprilTagVisionMenu);
        startPosMenu.addChoice("Start Position Backstage", StartPos.BACKSTAGE, false, useAprilTagVisionMenu);

        useAprilTagVisionMenu.addChoice("Use Vision", true, true, parkPosMenu);
        useAprilTagVisionMenu.addChoice("No Vision", false, false, parkPosMenu);

        parkPosMenu.addChoice("Park at Corner", ParkPos.CORNER, true);
        parkPosMenu.addChoice("Park at Center", ParkPos.CENTER, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(delayMenu);
        //
        // Fetch choices.
        //
        autoChoices.delay = delayMenu.getCurrentValue();
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.startPos = startPosMenu.getCurrentChoiceObject();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.useAprilTagVision = useAprilTagVisionMenu.getCurrentChoiceObject();
        autoChoices.parkPos = parkPosMenu.getCurrentChoiceObject();
        autoChoices.xTarget = xTargetMenu.getCurrentValue();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.turnTarget = turnTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Auto Choices: %s", autoChoices);
    }   //doAutoChoicesMenus

}   //class FtcAuto
