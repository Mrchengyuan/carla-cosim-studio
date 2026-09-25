// Thin cross-platform layer: TCP sockets and launching the Python backend.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace plat {

using Socket = std::intptr_t;
constexpr Socket kInvalidSocket = -1;

bool NetInit();
Socket TcpConnect(const std::string& host, int port, std::string& err);
bool SendAll(Socket s, const std::string& data);
int Recv(Socket s, char* buf, int len);  // <= 0: closed or error
void CloseSocket(Socket s);

struct Process {
  std::intptr_t handle = 0;  // HANDLE on Windows, pid on POSIX
  bool valid() const { return handle != 0; }
};

// Starts argv[0] with arguments, working directory cwd, stdout+stderr to log_path.
bool Spawn(const std::vector<std::string>& argv, const std::string& cwd,
           const std::string& log_path, Process& out, std::string& err);
bool IsAlive(const Process& p);
void Kill(Process& p);

std::string ExecutableDir();
// Command-line arguments as UTF-8 (on Windows argv is in the ANSI code page,
// which breaks non-ASCII paths; there the wide command line is used instead).
std::vector<std::string> Utf8Args(int argc, char** argv);
bool FileExists(const std::string& path);

}  // namespace plat
