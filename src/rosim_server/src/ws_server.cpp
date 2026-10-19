// Boost.Beast HTTP + WebSocket server. Serves the debug/production pages and
// vendored three.js assets, and fans out JSON text frames / binary JPEG frames
// to subscribed clients. All broadcast methods are safe from any thread.

#include "rosim_server/ws_server.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core/buffers_range.hpp>

#include <deque>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace rosim::server
{

namespace
{
/// One outgoing frame queued to a client (binary JPEG or text JSON).
struct OutMessage
{
  enum Kind
  {
    Binary,
    Text
  };
  Kind kind;
  std::shared_ptr<const std::string> data;
};

/// Who a broadcast should reach. Stream = binary JPEG subscribers, Scene = JSON
/// geometry subscribers (the debug WebGL viewer), Workpiece = clients that want
/// the part geometry only (the production viewer; debug scene subscribers get
/// it too), All = every client.
enum class Target
{
  All,
  Stream,
  Scene,
  Workpiece
};

class WsSession;
class HttpSession;
}  // namespace

// ---------------------------------------------------------------------------
// WsServer::Impl — owns the acceptor, the I/O thread and the live sessions.
// Declared before the session classes (which hold Impl*) and defined after
// them (so the member functions can see their full types).
// ---------------------------------------------------------------------------
class WsServer::Impl
{
public:
  Impl(const std::string& address, unsigned short port, const std::string& web_root);
  ~Impl();

  bool start();
  void stop();

  void onSessionOpen(const std::shared_ptr<WsSession>& session);
  void onSessionClose(const std::shared_ptr<WsSession>& session);
  void onMessage(const std::string& text, WsServer::BinarySetter set_binary,
                 WsServer::BinarySetter set_scene,
                 WsServer::BinarySetter set_workpiece);

  void broadcastBinary(const std::vector<uint8_t>& data);
  void broadcastText(const std::string& text);
  void broadcastScene(const std::string& text);
  void broadcastWorkpiece(const std::string& text);
  size_t clientCount() const;
  size_t binarySubscriberCount() const;
  size_t sceneSubscriberCount() const;

  const std::string* asset(beast::string_view path) const;
  std::string mimeType(beast::string_view path) const;

  WsServer::MessageHandler message_handler_;
  WsServer::ConnectHandler connect_handler_;

private:
  std::string resolveKey(beast::string_view path) const;
  void doAccept();
  void broadcast(Target target, OutMessage::Kind kind, const std::string& data);
  void loadAssets();

  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  std::string web_root_;
  /// URL path (no leading slash) -> file body, loaded at startup from web_root.
  std::map<std::string, std::string> assets_;

  std::thread thread_;
  mutable std::mutex sessions_mutex_;
  std::set<std::shared_ptr<WsSession>> sessions_;
};

namespace
{

/// A single WebSocket client connection. Owns its socket-backed websocket
/// stream, a serialized outgoing queue (Beast allows one outstanding write per
/// stream) and the read loop that feeds text frames to the message handler.
class WsSession : public std::enable_shared_from_this<WsSession>
{
public:
  WsSession(tcp::socket socket, WsServer::Impl* owner)
    : ws_(std::move(socket)), owner_(owner)
  {
    // MJPEG frames (~25 KB) must arrive as single WS messages. Beast fragments
    // writes larger than write_buffer_bytes (4 KB) by default; disable that so
    // each JPEG is one frame — simpler for the browser and the wire parser.
    ws_.auto_fragment(false);
  }

  /// Continue the WebSocket handshake from an already-read HTTP upgrade
  /// request, then start reading messages.
  template <typename Body>
  void run(http::request<Body> req)
  {
    ws_.async_accept(req, [self = shared_from_this()](beast::error_code ec) {
      if (ec)
        return;
      self->owner_->onSessionOpen(self);
      self->startRead();
    });
  }

  /// Enqueue a frame for this client. Thread-safe.
  void send(OutMessage::Kind kind, const std::string& data)
  {
    auto msg = std::make_shared<OutMessage>();
    msg->kind = kind;
    msg->data = std::make_shared<const std::string>(data);

    std::lock_guard<std::mutex> lock(write_mutex_);
    queue_.push_back(std::move(msg));
    if (writing_)
      return;
    writing_ = true;
    doWrite();
  }

  void close()
  {
    beast::error_code ec;
    ws_.close(websocket::close_code::normal, ec);
  }

  /// Toggle whether this client receives binary JPEG frames. Read from the
  /// render thread, written from the I/O thread, so use an atomic.
  void setBinaryEnabled(bool enabled) { binary_enabled_.store(enabled); }
  bool binaryEnabled() const { return binary_enabled_.load(); }

  /// Toggle whether this client receives JSON scene frames (geometry + joint
  /// state). Same cross-thread rules as binary.
  void setSceneEnabled(bool enabled) { scene_enabled_.store(enabled); }
  bool sceneEnabled() const { return scene_enabled_.load(); }

  /// Toggle whether this client receives workpiece geometry only (production
  /// viewer). Same cross-thread rules as binary.
  void setWorkpieceEnabled(bool enabled) { workpiece_enabled_.store(enabled); }
  bool workpieceEnabled() const { return workpiece_enabled_.load(); }

private:
  void startRead()
  {
    ws_.async_read(buffer_, [self = shared_from_this()](beast::error_code ec,
                                                        std::size_t /*bytes*/) {
      if (ec)
      {
        self->owner_->onSessionClose(self);
        return;
      }
      self->owner_->onMessage(beast::buffers_to_string(self->buffer_.data()),
                              [self](bool enabled) { self->setBinaryEnabled(enabled); },
                              [self](bool enabled) { self->setSceneEnabled(enabled); },
                              [self](bool enabled) { self->setWorkpieceEnabled(enabled); });
      self->buffer_.consume(self->buffer_.size());
      self->startRead();
    });
  }

  void doWrite()
  {
    // Caller holds write_mutex_. Pop the front message; when the queue drains,
    // clear the writing flag and stop (the next send() restarts the chain).
    if (queue_.empty())
    {
      writing_ = false;
      return;
    }
    auto msg = queue_.front();
    queue_.pop_front();

    ws_.text(msg->kind == OutMessage::Text);
    // Keep msg (which owns msg->data) alive until the async write completes:
    // beast's buffer only references the string, and a large scene frame
    // outlives this function — without the capture the buffer is freed and the
    // tail arrives as zeros (the browser then drops the whole scene JSON).
    ws_.async_write(asio::buffer(*msg->data),
                    [self = shared_from_this(), msg](beast::error_code ec, std::size_t /*bytes*/) {
                      if (ec)
                      {
                        std::lock_guard<std::mutex> lock(self->write_mutex_);
                        self->queue_.clear();
                        self->writing_ = false;
                        self->owner_->onSessionClose(self);
                        return;
                      }
                      std::lock_guard<std::mutex> lock(self->write_mutex_);
                      self->doWrite();
                    });
  }

  websocket::stream<tcp::socket> ws_;
  WsServer::Impl* owner_;

  beast::flat_buffer buffer_;
  std::mutex write_mutex_;
  std::deque<std::shared_ptr<OutMessage>> queue_;
  bool writing_ = false;
  std::atomic<bool> binary_enabled_{ false };
  std::atomic<bool> scene_enabled_{ false };
  std::atomic<bool> workpiece_enabled_{ false };
};

/// Content type for a static asset, keyed by file extension.
const char* mimeTypeFor(beast::string_view path)
{
  const auto dot = path.find_last_of('.');
  if (dot == beast::string_view::npos)
    return "application/octet-stream";
  const auto ext = path.substr(dot);
  if (ext == ".html")
    return "text/html; charset=utf-8";
  if (ext == ".js")
    return "text/javascript; charset=utf-8";
  if (ext == ".css")
    return "text/css; charset=utf-8";
  if (ext == ".svg")
    return "image/svg+xml";
  return "application/octet-stream";
}

/// Reads the initial HTTP request and routes it: a WebSocket upgrade becomes a
/// WsSession (taking over the socket), anything else is served a static file.
class HttpSession : public std::enable_shared_from_this<HttpSession>
{
public:
  HttpSession(tcp::socket socket, WsServer::Impl* owner)
    : socket_(std::move(socket)), owner_(owner)
  {
  }

  void run()
  {
    http::async_read(socket_, buffer_, req_,
                     [self = shared_from_this()](beast::error_code ec, std::size_t /*bytes*/) {
                       if (ec)
                         return;
                       if (websocket::is_upgrade(self->req_))
                       {
                         // Hand the socket to a WebSocket session.
                         auto ws = std::make_shared<WsSession>(std::move(self->socket_),
                                                               self->owner_);
                         ws->run(std::move(self->req_));
                       }
                       else
                       {
                         self->serve();
                       }
                     });
  }

private:
  void serve()
  {
    const std::string* body = owner_->asset(req_.target());

    http::response<http::string_body> res{ http::status::ok, req_.version() };
    if (!body)
    {
      res.result(http::status::not_found);
      res.set(http::field::content_type, "text/plain");
      res.body() = "not found";
    }
    else
    {
      res.set(http::field::content_type, owner_->mimeType(req_.target()));
      res.body() = *body;
    }
    res.prepare_payload();

    auto sp = std::make_shared<http::response<http::string_body>>(std::move(res));
    http::async_write(socket_, *sp,
                      [self = shared_from_this(), sp](beast::error_code ec, std::size_t) {
                        (void)ec;
                        beast::error_code ignored;
                        self->socket_.shutdown(tcp::socket::shutdown_send, ignored);
                      });
  }

  tcp::socket socket_;
  WsServer::Impl* owner_;
  beast::flat_buffer buffer_;
  http::request<http::string_body> req_;
};

}  // namespace

// ---------------------------------------------------------------------------
// WsServer::Impl implementation
// ---------------------------------------------------------------------------
WsServer::Impl::Impl(const std::string& address, unsigned short port,
                     const std::string& web_root)
  : ioc_(), acceptor_(ioc_), web_root_(web_root)
{
  loadAssets();
  tcp::resolver resolver(ioc_);
  tcp::endpoint endpoint = *resolver.resolve(address, std::to_string(port)).begin();
  acceptor_.open(endpoint.protocol());
  acceptor_.set_option(asio::socket_base::reuse_address(true));
  acceptor_.bind(endpoint);
  acceptor_.listen(asio::socket_base::max_listen_connections);
}

WsServer::Impl::~Impl()
{
  stop();
}

bool WsServer::Impl::start()
{
  if (thread_.joinable())
    return true;
  doAccept();
  thread_ = std::thread([this]() { ioc_.run(); });
  return true;
}

void WsServer::Impl::stop()
{
  if (!thread_.joinable())
    return;
  // Close every session (each unregisters itself), then stop the acceptor.
  std::vector<std::shared_ptr<WsSession>> sessions;
  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (const auto& s : sessions_)
      sessions.push_back(s);
  }
  for (auto& s : sessions)
    s->close();

  beast::error_code ec;
  acceptor_.close(ec);
  ioc_.stop();
  if (thread_.joinable())
    thread_.join();
}

void WsServer::Impl::onSessionOpen(const std::shared_ptr<WsSession>& session)
{
  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.insert(session);
  }
  if (connect_handler_)
    connect_handler_();
}

void WsServer::Impl::onSessionClose(const std::shared_ptr<WsSession>& session)
{
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  sessions_.erase(session);
}

void WsServer::Impl::onMessage(const std::string& text, WsServer::BinarySetter set_binary,
                               WsServer::BinarySetter set_scene,
                               WsServer::BinarySetter set_workpiece)
{
  if (message_handler_)
    message_handler_(text, std::move(set_binary), std::move(set_scene),
                     std::move(set_workpiece));
}

void WsServer::Impl::broadcastBinary(const std::vector<uint8_t>& data)
{
  broadcast(Target::Stream, OutMessage::Binary, std::string(data.begin(), data.end()));
}

void WsServer::Impl::broadcastText(const std::string& text)
{
  broadcast(Target::All, OutMessage::Text, text);
}

void WsServer::Impl::broadcastScene(const std::string& text)
{
  broadcast(Target::Scene, OutMessage::Text, text);
}

void WsServer::Impl::broadcastWorkpiece(const std::string& text)
{
  broadcast(Target::Workpiece, OutMessage::Text, text);
}

size_t WsServer::Impl::clientCount() const
{
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  return sessions_.size();
}

size_t WsServer::Impl::binarySubscriberCount() const
{
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  size_t count = 0;
  for (const auto& s : sessions_)
    if (s->binaryEnabled())
      ++count;
  return count;
}

size_t WsServer::Impl::sceneSubscriberCount() const
{
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  size_t count = 0;
  for (const auto& s : sessions_)
    if (s->sceneEnabled())
      ++count;
  return count;
}

std::string WsServer::Impl::resolveKey(beast::string_view path) const
{
  // Strip the leading slash, map page aliases to their file, reject traversal.
  if (path.empty() || path.front() != '/')
    return {};
  std::string key(path.substr(1));
  if (key.empty())
    key = "index.html";
  if (key == "production")
    key = "production.html";
  if (key.find("..") != std::string::npos)
    return {};
  return key;
}

const std::string* WsServer::Impl::asset(beast::string_view path) const
{
  const std::string key = resolveKey(path);
  if (key.empty())
    return nullptr;
  const auto it = assets_.find(key);
  return it == assets_.end() ? nullptr : &it->second;
}

std::string WsServer::Impl::mimeType(beast::string_view path) const
{
  // Content type follows the resolved file key, so "/" and "/production" get
  // text/html (their keys end in .html), not octet-stream.
  const std::string key = resolveKey(path);
  return mimeTypeFor(key.empty() ? beast::string_view(path) : beast::string_view(key));
}

void WsServer::Impl::doAccept()
{
  acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
    if (ec)
      return;
    std::make_shared<HttpSession>(std::move(socket), this)->run();
    doAccept();
  });
}

void WsServer::Impl::broadcast(Target target, OutMessage::Kind kind, const std::string& data)
{
  std::vector<std::shared_ptr<WsSession>> sessions;
  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (const auto& s : sessions_)
      sessions.push_back(s);
  }
  for (auto& s : sessions)
  {
    bool deliver = false;
    switch (target)
    {
      case Target::All:
        deliver = true;
        break;
      case Target::Stream:
        deliver = s->binaryEnabled();
        break;
      case Target::Scene:
        deliver = s->sceneEnabled();
        break;
      case Target::Workpiece:
        deliver = s->sceneEnabled() || s->workpieceEnabled();
        break;
    }
    if (deliver)
      s->send(kind, data);
  }
}

void WsServer::Impl::loadAssets()
{
  if (web_root_.empty())
    return;
  const auto load = [this](const std::string& url, const std::string& file) {
    std::ifstream in(web_root_ + "/" + file, std::ios::binary);
    if (!in)
      return;
    std::ostringstream ss;
    ss << in.rdbuf();
    assets_[url] = ss.str();
  };
  // The two pages (each also reachable via an alias resolved in asset()) plus
  // the vendored three.js viewer scripts for the debug WebGL page.
  load("index.html", "index.html");
  load("production.html", "production.html");
  load("viewer_common.js", "viewer_common.js");
  load("vendor/three.min.js", "vendor/three.min.js");
  load("vendor/OrbitControls.js", "vendor/OrbitControls.js");
}

// ---------------------------------------------------------------------------
// WsServer facade
// ---------------------------------------------------------------------------
WsServer::WsServer(const std::string& address, unsigned short port,
                   const std::string& web_root)
  : impl_(std::make_unique<Impl>(address, port, web_root)), port_(port)
{
}

WsServer::~WsServer() = default;

void WsServer::setMessageHandler(MessageHandler handler)
{
  impl_->message_handler_ = std::move(handler);
}

void WsServer::setConnectHandler(ConnectHandler handler)
{
  impl_->connect_handler_ = std::move(handler);
}

bool WsServer::start()
{
  return impl_->start();
}

void WsServer::stop()
{
  impl_->stop();
}

void WsServer::broadcastBinary(const std::vector<uint8_t>& data)
{
  impl_->broadcastBinary(data);
}

void WsServer::broadcastText(const std::string& text)
{
  impl_->broadcastText(text);
}

void WsServer::broadcastScene(const std::string& text)
{
  impl_->broadcastScene(text);
}

void WsServer::broadcastWorkpiece(const std::string& text)
{
  impl_->broadcastWorkpiece(text);
}

size_t WsServer::clientCount() const
{
  return impl_->clientCount();
}

size_t WsServer::binarySubscriberCount() const
{
  return impl_->binarySubscriberCount();
}

size_t WsServer::sceneSubscriberCount() const
{
  return impl_->sceneSubscriberCount();
}

}  // namespace rosim::server
