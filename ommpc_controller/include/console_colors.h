#ifndef MINISNAP_CONSOLE_COLORS_H
#define MINISNAP_CONSOLE_COLORS_H
namespace minisnap_console {
// Same normal colors/reset as sunray_common/common_lib/sunray_logger.h
// LOG_GREEN / LOG_CYAN / LOG_RESET. Do not pull in its global logger implementation:
// Logger::info additionally forces bold and underline.
constexpr const char* green = "\033[32m";
constexpr const char* cyan = "\033[36m";
constexpr const char* reset = "\033[0m";
}
#endif
