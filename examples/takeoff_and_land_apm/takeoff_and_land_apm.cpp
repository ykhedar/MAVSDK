//
// Simple example to demonstrate how takeoff and land using MAVSDK.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>


#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void test_set_rates(mavsdk::Telemetry& _aTelemetry);
void test_subscribe(mavsdk::Telemetry& _aTelemetry);
void test_takeoff_land( mavsdk::Telemetry& _aTelemetry,
                        mavsdk::Action& _aAction);

void test_mission_raw( mavsdk::Telemetry& _aTelemetry,
                        mavsdk::Action& _aAction,
                        mavsdk::MissionRaw& _aMissionRaw);

void test_mission_raw_import( mavsdk::Telemetry& _aTelemetry,
                        mavsdk::Action& _aAction,
                        mavsdk::MissionRaw& _aMissionRaw);

MissionRaw::MissionItem create_mission_item(uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
                                            float _param1, float _param2, float _param3, float _param4, 
                                            float _x, float _y, float _z, uint32_t _mission_type);

std::vector<MissionRaw::MissionItem> create_mission_raw();

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system};
    auto action = Action{system};
    auto param = Param{system};
    auto mission_raw = MissionRaw{system};

    //first get all params from the autopilot and print them.
    //Param::AllParams _all_params = param.get_all_params();
    //std::cout << "Params: " << _all_params << " m\n";

    // Check for Parameter Setting for all parameters which were obtained using get_all_params()

    //test_set_rates(telemetry);
    //test_subscribe(telemetry);
    test_mission_raw( telemetry, action, mission_raw);
    // test_takeoff_land( telemetry, action);

    // // Set up callback to monitor altitude while the vehicle is in flight
    // telemetry.subscribe_position([](Telemetry::Position position) {
    //     std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    // });

    return 0;
}


void test_subscribe(mavsdk::Telemetry& _aTelemetry)
{
    _aTelemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Position: " << position << " m\n";
    });

    _aTelemetry.subscribe_position_velocity_ned([](Telemetry::PositionVelocityNed position) {
        std::cout << "PositionVelocityNed: " << position.position << " m\n";
    });

    _aTelemetry.subscribe_home([](Telemetry::Position position) {
        std::cout << "Home: " << position << " m\n";
    });

    _aTelemetry.subscribe_in_air([](bool position) {
        std::cout << "InAir: " << position << " m\n";
    });

    _aTelemetry.subscribe_status_text([](Telemetry::StatusText position) {
        std::cout << "StatusText: " << position << " m\n";
    });

    _aTelemetry.subscribe_armed([](bool position) {
        std::cout << "Armed: " << position << " m\n";
    });

    _aTelemetry.subscribe_attitude_quaternion([](Telemetry::Quaternion position) {
        std::cout << "Att Quat: " << position << " m\n";
    });

    _aTelemetry.subscribe_attitude_euler([](Telemetry::EulerAngle position) {
        std::cout << "Att Euler: " << position << " m\n";
    });

    _aTelemetry.subscribe_attitude_angular_velocity_body([](Telemetry::AngularVelocityBody position) {
        std::cout << "Att Angular Velocity: " << position << " m\n";
    });

    _aTelemetry.subscribe_attitude_euler([](Telemetry::EulerAngle position) {
        std::cout << "Att Euler: " << position << " m\n";
    });

    _aTelemetry.subscribe_attitude_euler([](Telemetry::EulerAngle position) {
        std::cout << "Att Euler: " << position << " m\n";
    });

    _aTelemetry.subscribe_velocity_ned([](Telemetry::VelocityNed position) {
        std::cout << "Velocity NED: " << position << " m\n";
    });

    _aTelemetry.subscribe_imu([](Telemetry::Imu position) {
        std::cout << "IMU: " << position << " m\n";
    });

    _aTelemetry.subscribe_scaled_imu([](Telemetry::Imu position) {
        std::cout << "Scaled IMU: " << position << " m\n";
    });


    _aTelemetry.subscribe_raw_imu([](Telemetry::Imu position) {
        std::cout << "Raw IMU: " << position << " m\n";
    });

    _aTelemetry.subscribe_gps_info([](Telemetry::GpsInfo position) {
        std::cout << "GPS Info: " << position << " m\n";
    });

    _aTelemetry.subscribe_raw_gps([](Telemetry::RawGps position) {
        std::cout << "Raw GPS Info: " << position << " m\n";
    });

    _aTelemetry.subscribe_battery([](Telemetry::Battery position) {
        std::cout << "Battery: " << position << " m\n";
    });


    _aTelemetry.subscribe_flight_mode([](Telemetry::FlightMode position) {
        std::cout << "Flight Mode: " << position << " m\n";
    });

    _aTelemetry.subscribe_health([](Telemetry::Health position) {
        std::cout << "Health: " << position << " m\n";
    });

    _aTelemetry.subscribe_health_all_ok([](bool position) {
        std::cout << "Health All Ok: " << position << " m\n";
    });

    _aTelemetry.subscribe_landed_state([](Telemetry::LandedState position) {
        std::cout << "Landed State: " << position << " m\n";
    });

    _aTelemetry.subscribe_rc_status([](Telemetry::RcStatus position) {
        std::cout << "RC Status: " << position << " m\n";
    });

    _aTelemetry.subscribe_unix_epoch_time([](uint64_t position) {
        std::cout << "Unix Epoch: " << position << " m\n";
    });

    _aTelemetry.subscribe_odometry([](Telemetry::Odometry position) {
        std::cout << "Odometry: " << position << " m\n";
    });

    _aTelemetry.subscribe_distance_sensor([](Telemetry::DistanceSensor position) {
        std::cout << "Distance sensor: " << position << " m\n";
    });

}


void test_set_rates(mavsdk::Telemetry& _aTelemetry)
{
    {
        const auto set_rate_result = _aTelemetry.set_rate_position_velocity_ned(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_position_velocity_ned failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_position(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_position failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_home(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_home failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_in_air(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_in_air failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_landed_state(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_landed_state failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_attitude(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_attitude failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_velocity_ned(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_velocity_ned failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_imu(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_imu failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_ground_truth(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_ground_truth failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_gps_info(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_gps_info failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_battery(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_battery failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_rc_status(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_rc_status failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_position(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_position failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_unix_epoch_time(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_unix_epoch_time failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_distance_sensor(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_distance_sensor failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_odometry(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_odometry failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_actuator_output_status(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_actuator_output_status failed: " << set_rate_result << '\n';
        }
    }
    {
        const auto set_rate_result = _aTelemetry.set_rate_actuator_control_target(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting set_rate_actuator_control_target failed: " << set_rate_result << '\n';
        }
    }

}


void test_takeoff_land( mavsdk::Telemetry& _aTelemetry,
                        mavsdk::Action& _aAction)
{
    // Check until vehicle is ready to arm
    while (_aTelemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }
    std::cout << "Health: " << _aTelemetry.health();

    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';
    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = _aAction.arm();
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return;
    }
    std::cout << "Armed.\n" ;
    std::cout << "Taking off...\n";
    
    std::cout << "Setting takeoff altitude\n";
    _aAction.set_takeoff_altitude(10.0f);

    const Action::Result takeoff_result = _aAction.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return;
    }

    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';
    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));

    std::cout << "Landing...\n";
    const Action::Result land_result = _aAction.land();
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << '\n';
        return;
    }
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';

    // Check if vehicle is still in air
    while (_aTelemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';
    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished...\n";
}

void test_mission_raw_import( mavsdk::Telemetry& _aTelemetry,
                        mavsdk::Action& _aAction,
                        mavsdk::MissionRaw& _aMissionRaw)
{
    auto import_result =
        _aMissionRaw.import_qgroundcontrol_mission("./src/integration_tests/triangle.plan");

    std::cout << import_result.first; 
    std::cout << "Waiting for system to be ready";
    //std::cout << _aMissionRaw.

    while(!_aTelemetry.health_all_ok())
    {
        std::cout << "Waiting for system to be ready \n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    const Action::Result arm_result = _aAction.arm();
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';

    if (arm_result != Action::Result::Success) {
        std::cout << "Arming Result: " << arm_result << '\n';
        return;
    }
    std::cout << "Arming Result: " << arm_result << '\n';

    const MissionRaw::Result _uploadResult = _aMissionRaw.upload_mission(import_result.second.mission_items);
    std::cout << "Mission Items: " << import_result.second;
    const MissionRaw::Result _startResult =_aMissionRaw.start_mission();

    auto prom = std::promise<void>();
    auto fut = prom.get_future();

    _aMissionRaw.subscribe_mission_progress([&](MissionRaw::MissionProgress progress) {
        std::cout << "Progress: " << progress.current << "/" << progress.total;
        if (progress.current == progress.total) {
            _aMissionRaw.subscribe_mission_progress(nullptr);
            prom.set_value();
        }
    });

    fut.wait_for(std::chrono::seconds(120)) == std::future_status::ready;
    fut.get();

    // while(fut.wait_for(std::chrono::seconds(120)) != std::future_status::ready)

    const Action::Result _rtl_result = _aAction.return_to_launch();

    while (_aTelemetry.armed()) {
        std::cout << "Waiting for drone to be landed and disarmed.";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';

    sleep_for(seconds(3));
    std::cout << "Finished...\n";
}


void test_mission_raw( mavsdk::Telemetry& _aTelemetry,
                        mavsdk::Action& _aAction,
                        mavsdk::MissionRaw& _aMissionRaw)
{

    while(!_aTelemetry.health_all_ok())
    {
        std::cout << "Waiting for system to be ready \n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    

    
    std::promise<void> prom_changed{};
    std::future<void> fut_changed = prom_changed.get_future();

    std::atomic<bool> called_once{false};

    std::cout << "Subscribe for mission changed notification \n";
    _aMissionRaw.subscribe_mission_changed([&prom_changed, &called_once](bool) {
        bool flag = false;
        if (called_once.compare_exchange_strong(flag, true)) {
            prom_changed.set_value();
        }
    });

    std::vector<MissionRaw::MissionItem> mission_raw_items = create_mission_raw();
    _aMissionRaw.clear_mission();
    std::this_thread::sleep_for(std::chrono::seconds(10));

    {
        std::cout << "Uploading mission...\n";
        // We only have the upload_mission function asynchronous for now, so we wrap it using
        // std::future.
        std::promise<void> prom{};
        std::future<void> fut = prom.get_future();
        _aMissionRaw.upload_mission_async(mission_raw_items, [&prom](MissionRaw::Result result) {
            if (result == MissionRaw::Result::Success){
                prom.set_value();
                std::cout << "Mission uploaded. \n";
            }
            else {
                std::cout << "Mission Upload Failed.. \n";
            }
        });

        auto status = fut.wait_for(std::chrono::seconds(30));
        if (status == std::future_status::ready) {
            fut.get();
        }
    }

    std::cout << "Taking off...\n";
    
    std::cout << "Setting takeoff altitude\n";
    _aAction.set_takeoff_altitude(50.0f);

    const Action::Result arm_result = _aAction.arm();
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';

    if (arm_result != Action::Result::Success) {
        std::cout << "Arming Result: " << arm_result << '\n';
        return;
    }
    std::cout << "Arming Result: " << arm_result << '\n';
    
    const Action::Result takeoff_result = _aAction.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(30));
    std::cout << "Starting Mission \n ";

    const MissionRaw::Result _startResult =_aMissionRaw.start_mission();

    if (_startResult != MissionRaw::Result::Success) {
        std::cerr << "Starting mission failed: " << _startResult << '\n';
        return;
    }

    // while (!_aMissionRaw.).second) {
    //     sleep_for(seconds(1));
    // }

    auto prom = std::promise<void>();
    auto fut = prom.get_future();

    _aMissionRaw.subscribe_mission_progress([&](MissionRaw::MissionProgress progress) {
        std::cout << "Progress: " << progress.current << "/" << progress.total;
        if (progress.current == progress.total) {
            _aMissionRaw.subscribe_mission_progress(nullptr);
            prom.set_value();
        }
    });

    fut.wait_for(std::chrono::seconds(120)) == std::future_status::ready;
    fut.get();

    // while(fut.wait_for(std::chrono::seconds(120)) != std::future_status::ready)

    // const Action::Result _rtl_result = _aAction.return_to_launch();

    while (_aTelemetry.armed()) {
        std::cout << "Waiting for drone to be landed and disarmed. \n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Flight Mode is: " << _aTelemetry.flight_mode() << '\n';

    sleep_for(seconds(3));
    std::cout << "Finished...\n";
}



std::vector<MissionRaw::MissionItem> create_mission_raw()
{
    std::vector<MissionRaw::MissionItem> mission_raw_items;

    // #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    // cmds.add(Command( 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    //         Command(seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)

    // #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    // point1 = get_location_metres(aLocation, aSize, -aSize)
    // point2 = get_location_metres(aLocation, aSize, aSize)
    // point3 = get_location_metres(aLocation, -aSize, aSize)
    // point4 = get_location_metres(aLocation, -aSize, -aSize)
    // cmds.add(Command( 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    // cmds.add(Command( 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    // cmds.add(Command( 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    // cmds.add(Command( 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    // cmds.add(Command( 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    
    // Home = 47.398538996176875, 8.54554255, 483.37876752649856
    // Pt1 = 47.39776911820642, 8.545794816614517, 30
    // Pt2 = 47.39814478901126, 8.544659618054993, 30
    // Pt3 = 47.397800824620234, 8.544310052851216, 30
    // Pt4 = 47.39771545325316, 8.54576238283903, 30

    // _seq, _frame, _command, _current, _autocontinue, _param1, _param2, _param3, _param4, _x,  _y,  _z,  _mission_type
    // Add Home Position
    mission_raw_items.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, 47.397742, 8.545594, 488, 0));

    // Add Takeoff
    mission_raw_items.push_back(create_mission_item(1, 3, 22, 0, 1, 0, 0, 0, 0, 47.397742, 8.545594, 30, 0));

    // Add Mission Item 1-4
    mission_raw_items.push_back(create_mission_item(2, 3, 16, 0, 1, 0, 0, 0, 0, 47.39776911820642, 8.545794816614517, 30, 0));
    mission_raw_items.push_back(create_mission_item(3, 3, 16, 0, 1, 0, 0, 0, 0, 47.39814478901126, 8.544659618054993, 30, 0));
    mission_raw_items.push_back(create_mission_item(4, 3, 16, 0, 1, 0, 0, 0, 0, 47.397800824620234, 8.544310052851216, 30, 0));
    mission_raw_items.push_back(create_mission_item(5, 3, 16, 0, 1, 0, 0, 0, 0, 47.39771545325316, 8.54576238283903, 30, 0));

    // Return to Launch
    // mission_raw_items.push_back(create_mission_item(6, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0));


    // seq current frame cmd    p1          p2          p3 p4 
    // 0  1   0   16  0           0           0           0           -35.363262  149.165237  584.00000  1      Home         
    // 1  0   0   22  0.000000    0.000000    0.000000    0.000000    -35.361988  149.163753  00.000000  1      Takeoff
    // 2  0   0   16  0.000000    0.000000    0.000000    0.000000    -35.361992  149.163593  00.000000  1      Item 1
    // 3  0   0   16  0.000000    0.000000    0.000000    0.000000    -35.363812  149.163609  00.000000  1      Item 2
    // 4  0   0   16  0.000000    0.000000    0.000000    0.000000    -35.363768  149.166055  00.000000  1      Item 3
    // 5  0   0   16  0.000000    0.000000    0.000000    0.000000    -35.361835  149.166012  00.000000  1      Item 4
    // 6  0   0   16  0.000000    0.000000    0.000000    0.000000    -35.362150  149.165046  00.000000  1      Item 5

    return mission_raw_items;
}



MissionRaw::MissionItem create_mission_item(uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
                                            float _param1, float _param2, float _param3, float _param4, 
                                            float _x, float _y, float _z, 
                                            uint32_t _mission_type)
{
    MissionRaw::MissionItem new_raw_item_nav{};
    new_raw_item_nav.seq = _seq;
    new_raw_item_nav.frame = _frame; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    new_raw_item_nav.command = _command; // MAV_CMD_NAV_WAYPOINT
    new_raw_item_nav.current = _current;
    new_raw_item_nav.autocontinue = _autocontinue;
    new_raw_item_nav.param1 = _param1; // Hold
    new_raw_item_nav.param2 = _param2; // Accept Radius
    new_raw_item_nav.param3 = _param3; // Pass Radius
    new_raw_item_nav.param4 = _param4; // Yaw
    new_raw_item_nav.x = int32_t(std::round(_x * 1e7));
    new_raw_item_nav.y = int32_t(std::round(_y * 1e7));
    new_raw_item_nav.z = _z;
    new_raw_item_nav.mission_type = 0; // MAV_MISSION_TYPE_MISSION
    return new_raw_item_nav;
}


// struct MissionItem {
//         uint32_t seq{}; /**< @brief Sequence (uint16_t) */
//         uint32_t frame{}; /**< @brief The coordinate system of the waypoint (actually uint8_t) */
//         uint32_t command{}; /**< @brief The scheduled action for the waypoint (actually uint16_t) */
//         uint32_t current{}; /**< @brief false:0, true:1 (actually uint8_t) */
//         uint32_t autocontinue{}; /**< @brief Autocontinue to next waypoint (actually uint8_t) */
//         float param1{}; /**< @brief PARAM1, see MAV_CMD enum */
//         float param2{}; /**< @brief PARAM2, see MAV_CMD enum */
//         float param3{}; /**< @brief PARAM3, see MAV_CMD enum */
//         float param4{}; /**< @brief PARAM4, see MAV_CMD enum */
//         int32_t x{}; /**< @brief PARAM5 / local: x position in meters * 1e4, global: latitude in
//                         degrees * 10^7 */
//         int32_t y{}; /**< @brief PARAM6 / y position: local: x position in meters * 1e4, global:
//                         longitude in degrees *10^7 */
//         float z{}; /**< @brief PARAM7 / local: Z coordinate, global: altitude (relative or absolute,
//                       depending on frame) */
//         uint32_t mission_type{}; /**< @brief Mission type (actually uint8_t) */
// };







// frame type
// 0	MAV_FRAME_GLOBAL	Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
// 1	MAV_FRAME_LOCAL_NED	NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
// 2	MAV_FRAME_MISSION	NOT a coordinate frame, indicates a mission command.
// 3	MAV_FRAME_GLOBAL_RELATIVE_ALT	Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
// 4	MAV_FRAME_LOCAL_ENU	ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
// 5	MAV_FRAME_GLOBAL_INT	Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).
// 6	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT	Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home location.
// 7	MAV_FRAME_LOCAL_OFFSET_NED	NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.

// mission type
// 0	MAV_MISSION_TYPE_MISSION	Items are mission commands for main mission.
// 1	MAV_MISSION_TYPE_FENCE	Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
// 2	MAV_MISSION_TYPE_RALLY	Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.
// 255	MAV_MISSION_TYPE_ALL	Only used in MISSION_CLEAR_ALL to clear all mission types.




// mav cmd (mostly used: Takeoff, Waypoint, Loiter_Turns, RTL, Land)

// MAV_CMD_NAV_WAYPOINT (16 )

// [Command] Navigate to waypoint.
// Param (:Label)	Description	Values	Units
// 1: Hold	Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)	min:0	s
// 2: Accept Radius	Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)	min:0	m
// 3: Pass Radius	0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.		m
// 4: Yaw	Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).		deg
// 5: Latitude	Latitude		
// 6: Longitude	Longitude		
// 7: Altitude	Altitude		m

// MAV_CMD_NAV_LOITER_TIME (19 )
// [Command] Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.
// Param (:Label)	Description	Values	Units
// 1: Time	Loiter time (only starts once Lat, Lon and Alt is reached).	min:0	s
// 2: Heading Required	Leave loiter circle only once heading towards the next waypoint (0 = False)	min:0 max:1 increment:1	
// 3: Radius	Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.		m
// 4: Xtrack Location	Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.		
// 5: Latitude	Latitude		
// 6: Longitude	Longitude		
// 7: Altitude	Altitude


// MAV_CMD_NAV_RETURN_TO_LAUNCH (20 )

// [Command] Return to launch location
// Param (:Label)	Description
// 1	Empty
// 2	Empty
// 3	Empty
// 4	Empty
// 5	Empty
// 6	Empty
// 7	Empty

// MAV_CMD_NAV_LAND (21 )

// [Command] Land at location.
// Param (:Label)	Description	Values	Units
// 1: Abort Alt	Minimum target altitude if landing is aborted (0 = undefined/use system default).		m
// 2: Land Mode	Precision land mode.	PRECISION_LAND_MODE	
// 3	Empty.		
// 4: Yaw Angle	Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).		deg
// 5: Latitude	Latitude.		
// 6: Longitude	Longitude.		
// 7: Altitude	Landing altitude (ground level in current frame).		m


// MAV_CMD_NAV_TAKEOFF (22 )

// [Command] Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
// Param (:Label)	Description	Units
// 1: Pitch	Minimum pitch (if airspeed sensor present), desired pitch without sensor	deg
// 2	Empty	
// 3	Empty	
// 4: Yaw	Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).	deg
// 5: Latitude	Latitude	
// 6: Longitude	Longitude	
// 7: Altitude	Altitude	m