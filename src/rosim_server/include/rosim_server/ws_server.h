#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace rosim::server
{

/// \brief Minimal HTTP + WebSocket server for the debug/production web UI.
///
/// HTTP: serves the pages and static assets under web_root (debug page, the
/// production page, and the vendored three.js viewer scripts). WebSocket:
/// clients connect once, then the server pushes JSON control/status text frames
/// to everyone, binary JPEG frames (MJPEG over a binary WS message) only to
/// clients subscribed to the stream, and JSON scene frames (robot/workpiece
/// geometry + live joint state) only to clients subscribed to the scene — the
/// debug WebGL viewer. Clients send JSON text frames which are handed to the
/// message handler on the server's I/O thread.
///
/// All broadcast methods are thread-safe: the render thread and the ROS spin
/// thread may call them concurrently.
class WsServer
{
public:
  /// \brief Called (on the I/O thread) with each complete text frame a client
  /// sends (a JSON string) and three setters that toggle, for THAT client,
  /// whether it receives binary JPEG frames (set_binary), JSON scene frames
  /// (set_scene), and workpiece-only frames (set_workpiece) — all off by
  /// default. The workpiece target is for the production page, which needs the
  /// part geometry to show where each seam is but must not receive the robot
  /// scene or 12 Hz robot_state traffic. The setters are valid only for the
  /// duration of the call — call them synchronously, do not store them.
  using BinarySetter = std::function<void(bool enabled)>;
  using MessageHandler = std::function<void(const std::string& text,
                                            BinarySetter set_binary,
                                            BinarySetter set_scene,
                                            BinarySetter set_workpiece)>;
  /// \brief Called (on the I/O thread) when a new WebSocket client connects,
  /// so the app can immediately push the current state snapshot.
  using ConnectHandler = std::function<void()>;

  /// \param address bind address ("0.0.0.0" for any).
  /// \param port bind port.
  /// \param web_root absolute path to the web asset directory; may be empty to
  ///        disable the HTTP file server (WebSocket still works).
  WsServer(const std::string& address, unsigned short port,
           const std::string& web_root);
  ~WsServer();

  WsServer(const WsServer&) = delete;
  WsServer& operator=(const WsServer&) = delete;

  void setMessageHandler(MessageHandler handler);
  void setConnectHandler(ConnectHandler handler);

  /// Spawn the accept loop on a background thread. Returns false if the
  /// acceptor could not bind.
  bool start();

  /// Close all sessions and stop the I/O thread.
  void stop();

  /// Push a binary frame (JPEG) to every client subscribed to the stream.
  void broadcastBinary(const std::vector<uint8_t>& data);

  /// Push a text frame (JSON) to every connected client.
  void broadcastText(const std::string& text);

  /// Push a text frame (JSON) to every client subscribed to the scene.
  void broadcastScene(const std::string& text);

  /// Push a text frame (JSON) to every client subscribed to the workpiece
  /// (production page). Debug scene subscribers also receive it: they display
  /// the workpiece too.
  void broadcastWorkpiece(const std::string& text);

  unsigned short port() const { return port_; }
  size_t clientCount() const;

  /// Number of clients currently subscribed to the binary stream. The render
  /// loop uses this to skip GL capture when nobody is watching (production mode
  /// clients, or no clients at all).
  size_t binarySubscriberCount() const;

  /// Number of clients currently subscribed to the JSON scene (the debug WebGL
  /// viewer). The render loop uses this to skip link-transform serialization
  /// when nobody is watching.
  size_t sceneSubscriberCount() const;

  /// Opaque implementation handle (owns the I/O thread, acceptor, sessions).
  /// Public only so the implementation TU can define it; not part of the API.
  class Impl;

private:
  std::unique_ptr<Impl> impl_;
  unsigned short port_;
};

}  // namespace rosim::server
