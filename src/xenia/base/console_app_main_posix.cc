/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2021 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/base/console_app_main.h"
#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"

// TEMP (macos-arm64 Fable II bring-up): trace every way this process can leave
// main() so a "silent" disappearance stops being silent. Writes a one-line
// reason + backtrace to stderr and to /tmp/xenia_exit_trace.txt.
#include <cxxabi.h>
#include <execinfo.h>
#include <unistd.h>
#include <csignal>
#include <initializer_list>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>

namespace {

void xe_exit_trace_write(const char* why) {
  char stamp[32];
  time_t now = time(nullptr);
  struct tm tmv;
  localtime_r(&now, &tmv);
  strftime(stamp, sizeof(stamp), "%H:%M:%S", &tmv);

  void* bt[64];
  int n = backtrace(bt, 64);

  fprintf(stderr, "\n==== xenia exit trace [%s]: %s ====\n", stamp, why);
  backtrace_symbols_fd(bt, n, 2);
  fflush(stderr);

  FILE* f = fopen("/tmp/xenia_exit_trace.txt", "a");
  if (f) {
    fprintf(f, "\n==== xenia exit trace [%s]: %s ====\n", stamp, why);
    char** syms = backtrace_symbols(bt, n);
    if (syms) {
      for (int i = 0; i < n; i++) {
        fprintf(f, "  %s\n", syms[i]);
      }
      free(syms);
    }
    fflush(f);
    fclose(f);
  }
}

void xe_atexit_handler() { xe_exit_trace_write("atexit (std::exit or return from main)"); }
void xe_quick_exit_handler() { xe_exit_trace_write("at_quick_exit (std::quick_exit)"); }

[[noreturn]] void xe_terminate_handler() {
  const char* what = "std::terminate (uncaught exception?)";
  if (auto ep = std::current_exception()) {
    try {
      std::rethrow_exception(ep);
    } catch (const std::exception& e) {
      static char buf[512];
      snprintf(buf, sizeof(buf), "std::terminate: uncaught exception: %s", e.what());
      what = buf;
    } catch (...) {
      what = "std::terminate: uncaught non-std exception";
    }
  }
  xe_exit_trace_write(what);
  _exit(134);
}

void xe_signal_handler(int sig) {
  static const char* name = "?";
  switch (sig) {
    case SIGTERM: name = "SIGTERM"; break;
    case SIGINT: name = "SIGINT"; break;
    case SIGHUP: name = "SIGHUP"; break;
    case SIGQUIT: name = "SIGQUIT"; break;
    case SIGPIPE: name = "SIGPIPE"; break;
    case SIGABRT: name = "SIGABRT"; break;
    default: break;
  }
  xe_exit_trace_write(name);
  // Restore default and re-raise so the real disposition still happens.
  signal(sig, SIG_DFL);
  raise(sig);
}

void xe_install_exit_tracer() {
  static bool installed = false;
  if (installed) return;
  installed = true;
  std::atexit(xe_atexit_handler);
  std::at_quick_exit(xe_quick_exit_handler);
  std::set_terminate(xe_terminate_handler);
  for (int sig : {SIGTERM, SIGINT, SIGHUP, SIGQUIT, SIGPIPE, SIGABRT}) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = xe_signal_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(sig, &sa, nullptr);
  }
}

}  // namespace

int main(int argc, char** argv) {
  xe_install_exit_tracer();

  xe::ConsoleAppEntryInfo entry_info = xe::GetConsoleAppEntryInfo();

  if (!entry_info.transparent_options) {
    cvar::ParseLaunchArguments(argc, argv, entry_info.positional_usage,
                               entry_info.positional_options);
  }

  // Initialize logging. Needs parsed cvars.
  xe::InitializeLogging(entry_info.name);

  std::vector<std::string> args;
  for (int n = 0; n < argc; n++) {
    args.emplace_back(argv[n]);
  }

  int result = entry_info.entry_point(args);

  // xe::ShutdownLogging();

  return result;
}
