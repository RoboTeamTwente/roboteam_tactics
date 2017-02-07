#include "roboteam_tactics/practice_tests/TestSetup.h"

#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Commands.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"
#include <QtNetwork>
#include "ros/ros.h"

namespace rtt {
namespace practice {
     
const Config SetupBuilder::reset() {
    std::vector<TeamRobot> us, them;
    for (RobotID i = 0; i < 6; i++) {
        us.push_back({i, true});
        them.push_back({i, false});
    }
    return SetupBuilder::builder()->ignore_bots(us)->ignore_bots(them)
                                  ->stationary_ball()->with_ball({0,0})
                                  ->build();
} 
    
SetupBuilder* SetupBuilder::builder() {
    return new SetupBuilder();
}
    
SetupBuilder* SetupBuilder::with_bot(const TeamRobot& bot, const roboteam_utils::Position& pos, const roboteam_utils::Position& vel) {
    auto& s = (bot.our_team ? setup.us : setup.them)[bot.id];
    s.pos = pos;
    s.speed = vel;
    return this;
}

SetupBuilder* SetupBuilder::with_ball(const roboteam_utils::Vector2& pos) {
    setup.ballPos = pos;
    return this;
}

SetupBuilder* SetupBuilder::with_ball_vel(const roboteam_utils::Vector2& vel) {
    setup.ballSpeed = vel;
    return this;
}

SetupBuilder* SetupBuilder::stationary_ball() {
    setup.ballSpeed = roboteam_utils::Vector2();
    return this;
}

SetupBuilder* SetupBuilder::combine(const Config& other) {
    for (const auto& pair : other.us) {
        setup.us[pair.first] = pair.second;
    }
    for (const auto& pair : other.them) {
        setup.them[pair.first] = pair.second;
    }
    setup.ballPos = roboteam_utils::Vector2(other.ballPos);
    setup.ballSpeed = roboteam_utils::Vector2(other.ballSpeed);
    return this;
}
    
SetupBuilder* SetupBuilder::ignore_bot(const TeamRobot& bot) {
    double x = (.1 + .5 * bot.id) * (bot.our_team ? 1 : -1);
    return with_bot(bot, {x, 3.5, 270});
}    

SetupBuilder* SetupBuilder::ignore_bots(const std::vector<TeamRobot>& bots) {
    for (const TeamRobot& bot : bots) {
        ignore_bot(bot);
    }
    return this;
}

Config SetupBuilder::build() const {
    return setup;
}
 
namespace {
    QUdpSocket socket;
    
    void add_rep(grSim_Packet& packet, const std::pair<RobotID, Robot>& entry, bool we_are_yellow, bool our_team) {
        RobotID bot = entry.first;
        roboteam_utils::Position pos = entry.second.pos;
        //roboteam_utils::Position vel = entry.second.speed;
        grSim_RobotReplacement* rep = packet.mutable_replacement()->add_robots();
        rep->set_x(pos.x);
        rep->set_y(pos.y);
        rep->set_dir(pos.rot);
        rep->set_id(bot);
        rep->set_yellowteam(we_are_yellow == our_team);
    }
}
   
void send_setup_to_grsim(const Config& setup) {
    grSim_Packet packet;
    std::string our_color;
    get_PARAM_OUR_COLOR(our_color, true);
    bool we_are_yellow = our_color == "yellow";
    for (const auto& entry : setup.us) {
        add_rep(packet, entry, we_are_yellow, true);
    }
    for (const auto& entry : setup.them) {
        add_rep(packet, entry, we_are_yellow, false);
    }
    grSim_BallReplacement* ball = packet.mutable_replacement()->mutable_ball();
    ball->set_x(setup.ballPos.x);
    ball->set_y(setup.ballPos.y);
    ball->set_vx(setup.ballSpeed.x);
    ball->set_vy(setup.ballSpeed.y);
    
    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());

    // Send to IP address and port specified in grSim
    std::string grsim_ip = "127.0.0.1";
    int grsim_port = 20011;
    ros::param::get("grsim/ip", grsim_ip);
    ros::param::get("grsim/port", grsim_port);
    socket.writeDatagram(dgram, QHostAddress(QString::fromStdString(grsim_ip)), grsim_port);
}    
    
}
}