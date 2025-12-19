package igknighters.util.logging;

import monologue.Monologue;

/** A utility to log stuff initializing during "bootup" */
public class BootupLogger {
    private static final String println_prefix = "[Bootup] ";

    /**
     * Logs a message to the bootup log.
     *
     * <p>The bootup log is sent to console and to AKit logger
     *
     * @param message The message to log
     */
    public static synchronized void bootupLog(String message) {
        Monologue.log("/Bootup", message);

        if (true) {
            System.out.println(println_prefix + message);
        }
    }
}
