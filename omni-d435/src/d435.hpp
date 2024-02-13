#include <librealsense2/rs.hpp>
#include <vector>
#include <string>

class D435
{
public:
    D435(const std::string number);
    void update();
    std::string get_number();
    rs2::points get_points() { return points; };
    rs2::video_frame get_color() { return color; };

private:
    rs2::pipeline p;
    std::string serial_number;
    rs2::points points;
    rs2::video_frame color=rs2::video_frame(rs2::frame());
};