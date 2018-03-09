#pragma once

#include <Client.h>
#include <PubSubClient.h>
#include <Stream.h>
#include <stdarg.h>
#include <stdint.h>

// TODO should implement ESP8266 solution as well 
#if defined(ESP32)
#include <functional>
#include <memory>
#include <string>
#endif

#if defined(AVR)
#error "In AVR toolchain c++11 libraries could not used."
#endif

template <int N, typename client_t>
class MqttConnection {
   public:
    using handler_t = std::function<void(const std::string&)>;

    static void initialize(const std::string& host, const uint16_t port,
                   const std::string& clientId);

    MqttConnection(const std::string& host, const uint16_t port,
                   const std::string& clientId)
        : host_(host), clientId_(clientId), port_(port) {}

    ~MqttConnection() { client_.disconnect(); }

    void setDebugStream(Stream* stream);

    bool connect();

    void reconnect();

    bool isConnected();

    bool subscribe(const std::string& topic, handler_t handler);

    bool publish(const std::string& topic, const std::string& message);

    static std::shared_ptr<MqttConnection<N, client_t>> instance;
   
   private:
    struct HandlerMap {
        std::string topic;
        handler_t handler;
    };


    std::array<HandlerMap, N> handlers_;

    uint16_t handlerCount_ = 0;

    std::string host_;
    std::string clientId_;
    uint16_t port_;

    PubSubClient client_;

    client_t nativeClient_;

    std::shared_ptr<Stream> out_ = nullptr;

    void onMessageHandler(char* topic, uint8_t* payload, unsigned int length);
};

template <int N, typename C>
std::shared_ptr<MqttConnection<N, C>> MqttConnection<N, C>::instance = nullptr;

/**
 * IMPLEMENTATION
 */
template <int N, typename C>
inline void MqttConnection<N, C>::initialize(const std::string& host, const uint16_t port,
                   const std::string& clientId) {
    MqttConnection<N, C>::instance = std::make_shared<MqttConnection<N, C>>(host, port, clientId);
}

template <int N, typename C>
inline void MqttConnection<N, C>::setDebugStream(Stream* stream) {
    out_ = std::shared_ptr<Stream>(stream);
}

template <int N, typename C>
inline bool MqttConnection<N, C>::connect() {
    client_.setClient(nativeClient_);
    client_.setServer(host_.c_str(), port_);
    if (out_ != nullptr) {
        out_->printf("MQTT | Connection details\n       host: %s\n       port: %d\n       client id: %s\n",
                     host_.c_str(), port_, clientId_.c_str());
    }
    client_.setCallback(std::bind(&MqttConnection::onMessageHandler, this,
                                  std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3));

    reconnect();

    return true;
}

template <int N, typename C>
inline void MqttConnection<N, C>::reconnect() {
    // Loop until we're reconnected
    bool need_to_resubscribe = false;

    while (!client_.connected()) {
        need_to_resubscribe = true;
        if (out_ != nullptr) {
            out_->printf("MQTT | Attempting MQTT connection... ");
        }
        // Attempt to connect
        if (client_.connect(clientId_.c_str())) {
            if (out_ != nullptr) {
                out_->printf("connected\n");
            }
        } else {
            if (out_ != nullptr) {
                out_->printf("failed, rc=%d try again in 5 seconds\n",
                             client_.state());
            }
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }

    // also, we need to resubscribe every topic
    if( need_to_resubscribe ) {
        for(uint8_t i=0; i<handlerCount_; ++i) {
            auto handler = handlers_[i];
            if( client_.subscribe(handler.topic.c_str()) ) {
                if (out_ != nullptr) {
                    out_->printf("MQTT | Subscribed to: %s\n", handler.topic.c_str());
                }
            } else {
                if (out_ != nullptr) {
                    out_->printf("MQTT | Subsciption to: %s failed.\n", handler.topic.c_str());
                }
            }
        }
    }

    // send one loop
    client_.loop();
}

template <int N, typename C>
inline bool MqttConnection<N, C>::isConnected() {
    client_.loop();
    return client_.connected();
}

template <int N, typename C>
inline bool MqttConnection<N, C>::subscribe(const std::string& topic,
                                            handler_t handler) {
    if (handlerCount_ >= N) {
        return false;
    }
    handlers_[handlerCount_++] = {topic, handler};
    if (client_.subscribe(topic.c_str())) {
        if (out_ != nullptr) {
            out_->printf("MQTT | Subscribed to: %s\n", topic.c_str());
        }
        return true;
    }
    return false;
}

template <int N, typename C>
inline bool MqttConnection<N, C>::publish(const std::string& topic,
                                          const std::string& message) {
    if (client_.connected()) {
        bool rsp = client_.publish(topic.c_str(), message.c_str());
        if (rsp) {
            if (out_ != nullptr) {
                out_->printf("MQTT | Message published to: %s\n", topic.c_str());
            }
        } else {
            if (out_ != nullptr) {
                out_->printf("MQTT | Message publishing failed: %s\n", topic.c_str());
            }
        }
        return rsp;
    }
    return false;
}

template <int N, typename C>
inline void MqttConnection<N, C>::onMessageHandler(char* topic,
                                                   uint8_t* payload,
                                                   unsigned int length) {
    char buffer[length + 1];
    memcpy(buffer, payload, length);
    buffer[length] = 0;
    std::string p{buffer};
    for (auto& handler : this->handlers_) {
        if (strcmp(handler.topic.c_str(), topic) == 0) {
            handler.handler(p);
        }
    }
}