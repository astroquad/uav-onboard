#include "protocol/TelemetryMessage.hpp"

#include <iomanip>
#include <sstream>

namespace onboard::protocol {
namespace {

std::string escapeJson(const std::string& input)
{
    std::ostringstream out;
    for (const char ch : input) {
        switch (ch) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << ch;
            break;
        }
    }
    return out.str();
}

} // namespace

std::string buildTelemetryJson(const BringupTelemetry& telemetry)
{
    std::ostringstream json;
    json << std::fixed << std::setprecision(2);
    json << "{";
    json << "\"protocol_version\":1,";
    json << "\"type\":\"TELEMETRY\",";
    json << "\"seq\":" << telemetry.seq << ",";
    json << "\"timestamp_ms\":" << telemetry.timestamp_ms << ",";
    json << "\"mission\":{\"state\":\"IDLE\",\"elapsed_ms\":0},";
    json << "\"grid_pose\":{\"row\":-1,\"col\":-1,\"heading_deg\":0.0},";
    json << "\"marker_map\":[],";
    json << "\"vision\":{";
    json << "\"line_offset\":0.0,";
    json << "\"line_angle\":0.0,";
    json << "\"intersection_score\":0.0,";
    json << "\"marker_id\":-1,";
    json << "\"marker_offset_x\":0.0,";
    json << "\"marker_offset_y\":0.0";
    json << "},";
    json << "\"drone\":{";
    json << "\"altitude_m\":0.0,";
    json << "\"battery_voltage\":0.0,";
    json << "\"battery_pct\":0,";
    json << "\"armed\":false,";
    json << "\"flight_mode\":\"UNKNOWN\",";
    json << "\"failsafe\":false";
    json << "},";
    json << "\"safety\":{";
    json << "\"line_lost\":false,";
    json << "\"gcs_link_lost\":false,";
    json << "\"pixhawk_link_lost\":false,";
    json << "\"low_battery\":false";
    json << "},";
    json << "\"bringup\":{";
    json << "\"camera_status\":\"" << escapeJson(telemetry.camera_status) << "\",";
    json << "\"frame_width\":" << telemetry.frame_width << ",";
    json << "\"frame_height\":" << telemetry.frame_height << ",";
    json << "\"measured_fps\":" << telemetry.measured_fps << ",";
    json << "\"note\":\"" << escapeJson(telemetry.note) << "\"";
    json << "}";
    json << "}";
    return json.str();
}

} // namespace onboard::protocol
