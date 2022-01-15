#include "mavsdk.h"
#include "plugins/telemetry/telemetry.h"
#include "integration_test_helper.h"

using namespace mavsdk;

TEST(SitlTestDisabled, TelemetryGpsOrigin)
{
    Mavsdk mavsdk;

    ConnectionResult ret = mavsdk.add_udp_connection();
    ASSERT_EQ(ret, ConnectionResult::Success);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto system = mavsdk.systems().at(0);
    ASSERT_TRUE(system->has_autopilot());
    auto telemetry = Telemetry{system};

    while (!telemetry.health_all_ok()) {
        LogInfo() << "Waiting for system to be ready";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto result = telemetry.get_gps_global_origin();
    ASSERT_EQ(result.first, Telemetry::Result::Success);
    LogInfo() << result.second;
}
