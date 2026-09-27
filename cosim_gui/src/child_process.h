// Child programs of the remote launcher (ssh, the CarSim service, the GUI):
// output read line by line on a thread, stdin a pipe that stays open until
// CloseStdin(), and the whole process tree ended with the child. On Windows
// every child is in its own job object that kills it (and what it started,
// e.g. python.exe under py.exe) when the job is closed, so nothing outlives
// the launcher, even when the launcher itself is killed.
#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace plat {

class Child {
 public:
  Child() = default;
  ~Child();
  Child(const Child&) = delete;
  Child& operator=(const Child&) = delete;

  // argv[0]: the program (a full path; POSIX also searches PATH). capture:
  // stdout + stderr are read into Lines(); otherwise they are discarded.
  // hidden: no console window on Windows (console programs).
  bool Start(const std::vector<std::string>& argv, const std::string& cwd, bool capture, std::string& err);
  bool Started() const { return started_; }
  // Polls: false once it has ended (then ExitCode() is valid).
  bool Running();
  int ExitCode() const { return exit_code_; }
  long Pid() const { return pid_; }
  // Complete output lines since the last call (UTF-8; a line that is not
  // UTF-8 is converted from the Windows code page).
  std::vector<std::string> TakeLines();
  // EOF on its stdin (ssh then ends the session in order).
  void CloseStdin();
  // Asks it to end: WM_CLOSE to its windows on Windows, SIGTERM on POSIX.
  void RequestClose();
  // Ends it and everything it started, at once.
  void Kill();

 private:
  void ReadLoop();
  void Push(std::string& partial, const char* data, size_t n, bool flush);

  bool started_ = false;
  long pid_ = 0;
  int exit_code_ = -1;
  std::intptr_t process_ = 0;  // HANDLE (Windows)
  std::intptr_t job_ = 0;      // HANDLE (Windows)
  std::intptr_t out_ = -1;     // read end of stdout (HANDLE / fd)
  std::intptr_t in_ = -1;      // write end of stdin (HANDLE / fd)
  std::thread reader_;
  std::mutex mu_;
  std::deque<std::string> lines_;
};

// Quotes one argument for a Windows command line (CommandLineToArgvW rules).
std::string QuoteArg(const std::string& a);

// Opens a file, folder or URL with the system's default program.
bool OpenWithSystem(const std::string& target);

}  // namespace plat
