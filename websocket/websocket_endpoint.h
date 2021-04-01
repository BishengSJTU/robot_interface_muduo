#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <unistd.h>
#include "http_client.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;

class websocket_endpoint {
public:
    websocket_endpoint () {
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.clear_error_channels(websocketpp::log::elevel::all);
        m_endpoint.init_asio();
        m_endpoint.start_perpetual();
        m_thread.reset(new websocketpp::lib::thread(&client::run, &m_endpoint));
    }

    ~websocket_endpoint() {
        m_endpoint.stop_perpetual();
        if (m_status == "Open") {
            std::cout << "> Closing connection " << std::endl;
            websocketpp::lib::error_code ec;
            m_endpoint.close(hdl, websocketpp::close::status::going_away, "", ec);
            if (ec) {
                std::cout << "> Error closing connection " <<  ": "
                          << ec.message() << std::endl;
            }
        }
        m_thread->join();
    }

    void on_open(websocketpp::connection_hdl hdl) {
        m_status = "Open";
    }

    void on_fail(websocketpp::connection_hdl hdl) {
        m_status = "Failed";
    }

    void on_close(websocketpp::connection_hdl hdl) {
        m_status = "Closed";
    }

    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg) {
        std::string message = msg->get_payload();
        try {
            auto js = Json::parse(message);
            if(js["topic"] == "/bottom_byteinfo") {
                std::vector<int> data = js["msg"]["data"];
                if(data.size() != 18)
                    return;
                m_info.insert(std::pair<std::string, int>("power", data[9]));
            } else if(js["topic"] == "/move_base/result") {
                int status = js["msg"]["status"]["status"];
                m_info.insert(std::pair<std::string, int>("status", status));
            }
        } catch(...) {

        }
    }

    bool connect(std::string const & uri) {
        websocketpp::lib::error_code ec;
        client::connection_ptr con = m_endpoint.get_connection(uri, ec);
        if (ec) {
            std::cout << "> Connect initialization error: " << ec.message() << std::endl;
            return false;
        }

        hdl = con->get_handle();

        con->set_open_handler(websocketpp::lib::bind(
                &websocket_endpoint::on_open,
                this,
                websocketpp::lib::placeholders::_1
        ));
        con->set_fail_handler(websocketpp::lib::bind(
                &websocket_endpoint::on_fail,
                this,
                websocketpp::lib::placeholders::_1
        ));
        con->set_close_handler(websocketpp::lib::bind(
                &websocket_endpoint::on_close,
                this,
                websocketpp::lib::placeholders::_1
        ));
        con->set_message_handler(websocketpp::lib::bind(
                &websocket_endpoint::on_message,
                this,
                websocketpp::lib::placeholders::_1,
                websocketpp::lib::placeholders::_2
        ));

        m_endpoint.connect(con);
        return true;
    }

    void close(websocketpp::close::status::value code) {
        websocketpp::lib::error_code ec;
        m_endpoint.close(hdl, code, "", ec);
        if (ec) {
            std::cout << "> Error initiating close: " << ec.message() << std::endl;
        }
    }

    bool send(std::string message) {
        websocketpp::lib::error_code ec;
        m_endpoint.send(hdl, message, websocketpp::frame::opcode::text, ec);
        if (ec) {
            std::cout << "> Error sending message: " << ec.message() << std::endl;
            return false;
        }
        return true;
    }


    std::map<std::string, int>& get_set_info() {
        return m_info;
    }

    std::string const get_status() {
        return m_status;
    }


private:
    client m_endpoint;
    websocketpp::connection_hdl hdl;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
    std::string m_status;
    std::map<std::string, int> m_info;
};