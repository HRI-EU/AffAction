#ifndef AFF_EVENTCLIENT_H
#define AFF_EVENTCLIENT_H

#define ASIO_STANDALONE
#include <websocketpp/config/asio_no_tls_client.hpp> 
#include <websocketpp/client.hpp>

#include <Rcs_macros.h>
#include <json.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

namespace aff
{
typedef websocketpp::client<websocketpp::config::asio_client> wsclient;

/*! \brief Protocol-5 event producer: the C++ twin of event_sender.EventClient.
 *
 *  Connects to the agent's web frontend (ws://host:8443/ws), registers with an
 *  `event_hello`, and streams `event` frames into its EventCollector. The agent
 *  must run with --webinput and without --no_events, or the handshake is
 *  refused with an error frame saying exactly that.
 */
class EventClient
{
public:
  static constexpr size_t MAX_QUEUE = 256;
  static constexpr double READY_TIMEOUT_S = 5.0;
  static constexpr double RECONNECT_S = 3.0;

  EventClient(std::string uri_, std::string producer_)
    : uri(std::move(uri_)), producer(std::move(producer_)),
      sessionId(newSessionId()), running(false), connected(false),
      published(0), delivered(0), dropped(0), sequence(0)
  {
  }

  ~EventClient()
  {
    stop();
  }

  void start()
  {
    running = true;
    worker = std::thread(&EventClient::run, this);
  }

  void stop()
  {
    running = false;
    if (worker.joinable())
    {
      worker.join();
    }
  }

  bool isConnected() const
  {
    return connected;
  }

  //! State handed over at registration, so the collector starts consistent.
  std::function<nlohmann::json()> snapshot;
  //! Speech and turn-lifecycle edges from the agent, for a local HUD. Optional.
  std::function<void(const nlohmann::json&)> onAgentEvent;

  /*! \brief Queue one event. Safe from ANY thread, never blocks, never throws.
   *
   *  Returns false when the queue is full, in which case the NEWEST is dropped:
   *  a deep backlog is stale perception, and a greeting for someone who has
   *  since left is worse than no greeting.
   *
   *  `source` must be one of the collector's closed vocabulary - face, hand,
   *  presence, identity, speech, agent, client - or the frame is rejected.
   *  `name` is free-form.
   */
  bool publish(const std::string& name, const std::string& source,
               const nlohmann::json& detail = nlohmann::json::object(),
               const std::string& person = "", int trackId = -1)
  {
    nlohmann::json ev =
    {
      {"type", "event"}, {"name", name}, {"source", source},
      {"t", monotonic()},           // our own origin, uncorrected - see below
      {"person", person}, {"track_id", trackId}, {"detail", detail}
    };

    std::lock_guard<std::mutex> lock(qMtx);
    if (queue.size() >= MAX_QUEUE)
    {
      dropped++;
      return false;
    }
    queue.push_back(std::move(ev));
    published++;
    return true;
  }

private:

  // ANY monotonic origin works. The collector measures the offset between its
  // clock and ours during the handshake and corrects every later `t` itself;
  // pre-correcting here would do the same arithmetic twice. steady_clock's
  // epoch being unrelated to Python's time.monotonic() is therefore a non-issue.
  static double monotonic()
  {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
  }

  static std::string newSessionId()
  {
    std::random_device rd;
    std::ostringstream os;
    os << std::hex << rd() << rd();
    return os.str();
  }

  size_t discardBacklog()
  {
    std::lock_guard<std::mutex> lock(qMtx);
    const size_t n = queue.size();
    queue.clear();
    dropped += n;
    return n;
  }

  void run()
  {
    while (running)
    {
      wsclient endpoint;
      endpoint.clear_access_channels(websocketpp::log::alevel::all);
      endpoint.clear_error_channels(websocketpp::log::elevel::all);
      endpoint.init_asio();

      websocketpp::lib::error_code ec;
      wsclient::connection_ptr con = endpoint.get_connection(uri, ec);
      if (ec)
      {
        RLOG_CPP(1, "No connection to " << uri << ": " << ec.message());
        napUntilStopped(RECONNECT_S);
        continue;
      }

      std::atomic<bool> ready(false), dead(false);

      con->set_open_handler([this, con](websocketpp::connection_hdl)
      {
        // ORDER IS THE CONTRACT HERE, and getting it wrong loses a transition.
        // Everything queued while disconnected is dropped - it describes a room
        // that has moved on - but the cutoff must fall BEFORE the snapshot is
        // taken, never after. The snapshot states one instant; everything from
        // that instant on is the delta that keeps the collector consistent
        // with it. Discarding afterwards bins exactly the events that happened
        // during the handshake.
        const size_t stale = discardBacklog();
        nlohmann::json hello =
        {
          {"type", "event_hello"}, {"producer", producer},
          {"session_id", sessionId}, {"t", monotonic()}
        };
        if (snapshot)
        {
          hello["snapshot"] = snapshot();
        }
        con->send(hello.dump(), websocketpp::frame::opcode::text);
        RLOG_CPP(1, "Producer " << producer << " registered (dropped "
                 << stale << " stale)");
      });

      con->set_message_handler([this, &ready, &dead]
                               (websocketpp::connection_hdl, wsclient::message_ptr msg)
      {
        nlohmann::json f = nlohmann::json::parse(msg->get_payload(), nullptr, false);
        if (f.is_discarded() || !f.is_object())
        {
          return;
        }
        const std::string type = f.value("type", std::string());
        if (type == "event_ready")
        {
          ready = true;
        }
        else if (type == "agent_event" && onAgentEvent)
        {
          onAgentEvent(f);
        }
        else if (type == "error")
        {
          RLOG_CPP(0, "Collector refused: " << f.value("reason", std::string()));
          dead = true;
        }
      });

      con->set_fail_handler([&dead](websocketpp::connection_hdl)
      {
        dead = true;
      });
      con->set_close_handler([&dead](websocketpp::connection_hdl)
      {
        dead = true;
      });

      endpoint.connect(con);
      std::thread asio([&endpoint]()
      {
        endpoint.run();
      });

      // A handshake that never completes is a dead connection, not a slow one:
      // socket accepted, nothing answers, every event dropped while the log
      // cheerfully says connected. That state must reconnect, not sit there.
      const double deadline = monotonic() + READY_TIMEOUT_S;
      while (running && !ready && !dead && monotonic() < deadline)
      {
        napUntilStopped(0.01);
      }

      if (ready && !dead)
      {
        connected = true;
        drain(con, dead);
        connected = false;
      }
      else if (!dead)
      {
        RLOG_CPP(1, "No event_ready within " << READY_TIMEOUT_S
                 << "s; reconnecting");
      }

      endpoint.stop();
      asio.join();
      if (running)
      {
        napUntilStopped(RECONNECT_S);
      }
    }
  }

  void drain(wsclient::connection_ptr con, std::atomic<bool>& dead)
  {
    while (running && !dead)
    {
      nlohmann::json ev;
      {
        std::lock_guard<std::mutex> lock(qMtx);
        if (queue.empty())
        {
          ev = nullptr;
        }
        else
        {
          ev = std::move(queue.front());
          queue.pop_front();
        }
      }
      if (ev.is_null())
      {
        napUntilStopped(0.02);
        continue;
      }
      ev["sequence"] = ++sequence;   // diagnostics only; nothing retries a gap
      websocketpp::lib::error_code ec = con->send(ev.dump(),
                                                  websocketpp::frame::opcode::text);
      if (ec)
      {
        RLOG_CPP(1, "Producer socket lost (" << ec.message()
                 << "); reconnecting");
        dead = true;
        return;
      }
      delivered++;
    }
  }

  void napUntilStopped(double seconds) const
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }

  std::string uri, producer, sessionId;
  std::atomic<bool> running, connected;
  std::atomic<size_t> published, delivered, dropped;
  size_t sequence;
  std::deque<nlohmann::json> queue;
  mutable std::mutex qMtx;
  std::thread worker;
};








#include <ComponentBase.h>

class EventProducerComponent : public ComponentBase
{
public:
  EventProducerComponent(EntityBase* parent)
    : ComponentBase(parent), client("ws://localhost:8443/ws", "smile")
  {
    client.snapshot = [this]()
    {
      return nlohmann::json{{"person", ""}};
    };
    client.start();
    subscribe("ObjectGrasped", &EventProducerComponent::onGrasp);
    subscribe("PublishEvent", &EventProducerComponent::onEvent);
  }

  ~EventProducerComponent()
  {
    client.stop();
  }

private:
  void onGrasp(std::string object)   // called on the Rcs event loop - must not block
  {
    RLOG_CPP(0, "Publishing " << object);
    // `scene` is telemetry: it reaches the journal and the perception HUD, and
    // no rule matches it, so it never becomes a conversation turn. Publish a
    // `presence` event with detail {role, text} instead when Erna should speak.
    // Leave `person` empty - StateTable.observe adopts any person on any event.
    client.publish("object_grasped", "scene", {{"object", object}});
  }

  /*! \brief Forward an arbitrary Rcs event to the protocol-5 event stream.
   *
   *  `source` must be one of the collector's supported sources. `extra` may be
   *  a JSON object; plain text is preserved as {"extra": extra}.
   */
  void onEvent(std::string source, std::string name, std::string extra)
  {
    RLOG_CPP(0, "PublishEvent: source='" << source
             << "' name='" << name << "' extra='" << extra << "'");

    nlohmann::json detail = nlohmann::json::parse(extra, nullptr, false);
    if (detail.is_discarded() || !detail.is_object())
    {
      detail = {{"extra", extra}};
    }

    if (!client.publish(name, source, detail))
    {
      RLOG_CPP(1, "Failed to queue event " << source << "/" << name);
    }
  }

  aff::EventClient client;
};





}   // namespace aff

#endif
