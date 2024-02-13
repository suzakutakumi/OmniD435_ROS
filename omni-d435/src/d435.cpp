#include "d435.hpp"
#include <iostream>

// シリアルナンバーを受け取り、初期化する
D435::D435(const std::string number)
{
    rs2::config conf;
    try
    {
        conf.enable_device(number);
        serial_number = number;
        conf.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        conf.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
        conf.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

        p.start(conf);
    }
    catch (const rs2::error &e)
    {
        // conf.disable_all_streams();
        throw e;
    }
}

void D435::update()
{
    rs2::frameset frames = p.wait_for_frames();

    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    color = aligned_frames.first(RS2_STREAM_COLOR);
    if (!color)
        color = frames.get_infrared_frame();

    rs2::pointcloud pc;
    pc.map_to(color);

    rs2::depth_frame depth = aligned_frames.get_depth_frame();

    rs2::decimation_filter filter;
    filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    points = pc.calculate(filter.process(depth));
    
}

std::string D435::get_number()
{
    return serial_number;
}
