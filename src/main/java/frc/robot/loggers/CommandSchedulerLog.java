package frc.robot.loggers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.invoke.MethodHandles;
import java.util.HashMap;
import java.util.stream.Collectors;

/**
 * Log the course of a command execution on Console, DataLog, or ShuffleBoard
 * 
 * <p>This class could be used this way:
<pre>
  // options for logging
  private boolean useConsole            = false;
  private boolean useDataLog            = true;
  private boolean useShuffleBoardLog    = false;
    configureCommandLogs(); // do early on otherwise log not ready for first commands

  @SuppressWarnings("resource")
  public void configureCommandLogs()
  {
      if (useConsole || useDataLog || useShuffleBoardLog) {
        schedulerLog = new CommandSchedulerLog(useConsole, useDataLog, useShuffleBoardLog);
        schedulerLog.logCommandInitialize();
        schedulerLog.logCommandInterrupt();
        schedulerLog.logCommandFinish();
        schedulerLog.logCommandExecute();  // Can (optionally) generate a lot of output        
      }
      else {
        new Alert("No logging", AlertType.kWarning).set(true);
      }
  }
</pre>
 */
public final class CommandSchedulerLog 
{
    private static final String m_fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    
    static
    {
        System.out.println("Loading: " + m_fullClassName);
    }

    private static final HashMap<String, Integer> m_currentCommands = new HashMap<String, Integer>();
    private static NetworkTable m_nt;    
    private static StringEntry m_initializeCommandLogEntry;
    private static StringEntry m_interruptCommandLogEntry;
    private static StringEntry m_finishCommandLogEntry;
    private static StringEntry m_executeCommandLogEntry;
    private static boolean m_useConsole;
    private static boolean m_useDataLog;
    private static boolean m_useShuffleBoardLog;


    private CommandSchedulerLog()
    {}

    /**
     * Command Event Loggers
     * 
     * <p>Set the command scheduler to log events for command initialize, interrupt, finish, and execute.
     * 
     * <p>Log to the Console/Terminal, ShuffleBoard or the WPILib DataLog tool.
     * 
     * <p>If ShuffleBoard is recording (start it manually), these events are added to the recording.
     * Convert recording to csv and they show nicely in Excel.
     * 
     * <p>If using DataLog tool, the recording is via NT so tell NT to send EVERYTHING to the DataLog.
     * Run DataLog tool to retrieve log from roboRIO and convert the log to csv.
     * 
     * <p>Note the comment in execute logging that only the first execute is logged unless changed.
     * 
     * <p>Note that use of the DataLog creates SmartDashboard/ShuffleBoard entries but are not the same
     * as use of the ShuffleBoardLog. The ShuffleBoardLog has event markers independent of the DataLog.
     * 
     * @param useConsole
     * @param useDataLog
     * @param useShuffleBoardLog
     */ 
    public static void config(boolean useConsole, boolean useDataLog, boolean useShuffleBoardLog)
    {
        m_useConsole = useConsole;
        m_useDataLog = useDataLog;
        m_useShuffleBoardLog = useShuffleBoardLog;

        // DataLog via NT so establish NT and the connection to DataLog
        if (useDataLog) {
            DataLogManager.logNetworkTables(true); // WARNING - this puts all NT to the DataLog
        }

        final String networkTableName = "Team10473";
        m_nt = NetworkTableInstance.getDefault().getTable(networkTableName);
        m_initializeCommandLogEntry = m_nt.getStringTopic("Commands/initialize").getEntry("");
        m_interruptCommandLogEntry = m_nt.getStringTopic("Commands/interrupt").getEntry("");
        m_finishCommandLogEntry = m_nt.getStringTopic("Commands/finish").getEntry("");
        m_executeCommandLogEntry = m_nt.getStringTopic("Commands/execute").getEntry("");   
        
        logCommandInitialize();
        logCommandInterrupt();
        logCommandFinish();
        logCommandExecute();
    }

    /**
     * Log commands that run the initialize method.
     */
    private static void logCommandInitialize()
    {
        CommandScheduler.getInstance().onCommandInitialize(
            (command) -> 
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String requirements = command.getRequirements().stream()
                    .map(subsystem -> subsystem.getClass().getSimpleName())
                    .collect(Collectors.joining(", ", "{", "}"));

                if(m_useConsole) {
                    System.out.println("Command initialized : " + key + " " + requirements);                    
                }
                if(m_useDataLog) {
                    m_initializeCommandLogEntry.set(key + " " + requirements);                    
                }
                if(m_useShuffleBoardLog) {
                    Shuffleboard.addEventMarker("Command initialized",
                        key + " " + requirements, EventImportance.kNormal);                    
                }

                m_currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that have been interrupted.
     */
    private static void logCommandInterrupt()
    {
        CommandScheduler.getInstance().onCommandInterrupt(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String runs = " after " + m_currentCommands.getOrDefault(key, 0) + " runs";

                if(m_useConsole) {
                    System.out.println("Command interrupted : " + key + runs);                    
                }
                if(m_useDataLog) {
                    m_interruptCommandLogEntry.set(key + runs);                    
                } 
                if(m_useShuffleBoardLog) {
                    Shuffleboard.addEventMarker("Command interrupted", key, EventImportance.kNormal);
                }

                m_currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that run the finish method.
     */
    private static void logCommandFinish()
    {
        CommandScheduler.getInstance().onCommandFinish(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String runs = " after " + m_currentCommands.getOrDefault(key, 0) + " runs";

                if(m_useConsole) {
                    System.out.println("Command finished : " + key + runs);                    
                }
                if(m_useDataLog) {
                    m_finishCommandLogEntry.set(key + runs);                    
                } 
                if(m_useShuffleBoardLog) {
                    Shuffleboard.addEventMarker("Command finished", key, EventImportance.kNormal);                    
                }

                m_currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that run the execute() method.
     * 
     * <p>This can generate a lot of events so logging is suppressed except for the first
     * occurrence of execute(). Total count of execute() is logged at command end.
     * 
     * <p>Recompile without the if/else to get all execute() logged.
     */
    private static void logCommandExecute()
    {
        CommandScheduler.getInstance().onCommandExecute(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();

                if(m_currentCommands.getOrDefault(key, 0) == 0) // suppress all but first execute
                {
                    if(m_useConsole) {
                        System.out.println("Command executed : " + key);                        
                    }
                    if(m_useDataLog) {
                        m_executeCommandLogEntry.set(key);             
                    }
                    if(m_useShuffleBoardLog) {
                        Shuffleboard.addEventMarker("Command executed", key, EventImportance.kNormal);                        
                    }

                    m_currentCommands.put(key, 1); // first time through count is 1
                }
                else
                    // Increment total count to log when the command ends.
                    m_currentCommands.put(key, m_currentCommands.get(key) + 1);
            }
        );
    }
}
