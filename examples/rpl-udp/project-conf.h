#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Logging - compile with DBG so we can enable via shell when needed.
 * Application calls log_set_level() at startup to lower to INFO/WARN.
 * Use shell "log <module> <level>" to raise back to DBG for debugging.
 */
#define LOG_CONF_LEVEL_RPL       LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_IPV6      LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_MAC       LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_FRAMER    LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_RADIO     LOG_LEVEL_DBG

#endif /* PROJECT_CONF_H_ */
