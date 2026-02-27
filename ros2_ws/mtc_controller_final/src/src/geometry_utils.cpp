#include "mtc_dynamic/geometry_utils.h"

// C++标准库，用于文件操作和字符串处理
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>


// ROS2 日志库
#include <rclcpp/logging.hpp>

// 其他数学和消息库
#include <limits>
#include <cmath>
#include <shape_msgs/msg/solid_primitive.hpp>

// 获取一个局部的日志记录器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("geometry_utils");

namespace mtc_dynamic {
namespace utils {

// ✅ 新增：一个简易的、只解析顶点数据的PLY文件解析器
// 这是一个内部辅助函数，所以我们用static关键字，并放在匿名命名空间中
namespace {
std::vector<std::array<double, 3>> parseSimplePlyVertices(const std::string& ply_filepath) {
    std::vector<std::array<double, 3>> vertices;
    std::ifstream file(ply_filepath);

    if (!file.is_open()) {
        RCLCPP_ERROR(LOGGER, "无法打开PLY文件: %s", ply_filepath.c_str());
        return vertices;
    }

    std::string line;
    long vertex_count = 0;
    bool header_ended = false;

    // 1. 解析头部信息
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string keyword;
        ss >> keyword;

        if (keyword == "element") {
            std::string type;
            ss >> type;
            if (type == "vertex") {
                ss >> vertex_count;
            }
        } else if (keyword == "end_header") {
            header_ended = true;
            break; // 头部结束，准备读取顶点数据
        }
    }

    if (!header_ended || vertex_count == 0) {
        RCLCPP_ERROR(LOGGER, "PLY文件格式错误或没有找到顶点元素。");
        return vertices;
    }

    // 2. 读取顶点数据
    vertices.reserve(vertex_count);
    for (long i = 0; i < vertex_count && std::getline(file, line); ++i) {
        std::stringstream ss(line);
        double x, y, z;
        if (ss >> x >> y >> z) {
            vertices.push_back({x, y, z});
        } else {
            RCLCPP_WARN(LOGGER, "无法解析顶点行: %s", line.c_str());
        }
    }
    
    if (vertices.size() != static_cast<size_t>(vertex_count)) {
        RCLCPP_WARN(LOGGER, "警告：读取到的顶点数量 (%zu) 与头部声明的数量 (%ld) 不符。", vertices.size(), vertex_count);
    }

    return vertices;
}
} // 匿名命名空间结束

std::pair<double, double> calculateCylinderDimensions(const std::string& ply_filepath) {
    // 1. Parse vertices from the PLY file (same as before)
    std::vector<std::array<double, 3>> vertices = parseSimplePlyVertices(ply_filepath);

    if (vertices.empty()) {
        RCLCPP_ERROR(LOGGER, "Failed to parse vertices from PLY file, cannot calculate dimensions.");
        return {0.0, 0.0};
    }

    // --- Height Calculation (Robust Method) ---

    // 2. Collect all Z-coordinates into a vector
    std::vector<double> z_values;
    z_values.reserve(vertices.size());
    for (const auto& v : vertices) {
        z_values.push_back(v[2]);
    }

    // 3. Sort the Z-coordinates
    std::sort(z_values.begin(), z_values.end());

    // 4. Trim a percentage of outliers (e.g., 1%) from both ends
    const double trim_percentage = 0.01; // Trim 1% from the top and 1% from the bottom
    size_t trim_count = static_cast<size_t>(z_values.size() * trim_percentage);
    
    // Ensure we don't trim everything
    if (trim_count * 2 >= z_values.size()) {
        trim_count = 0; // Don't trim if the cloud is too small
    }

    // 5. Get the new min and max from the trimmed, sorted list
    double min_z = z_values[trim_count];
    double max_z = z_values[z_values.size() - 1 - trim_count];
    double height = max_z - min_z;


    // --- Diameter Calculation (Robust Method) ---
    
    // This part is computationally intensive. For a large number of points (N),
    // calculating all N*(N-1)/2 distances can be slow. A common optimization
    // is to calculate distances from the centroid, but for robustness,
    // we'll stick to the all-pairs check and make it more robust.

    // 6. Find the centroid (center of mass) of the point cloud in the XY plane
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const auto& v : vertices) {
        centroid_x += v[0];
        centroid_y += v[1];
    }
    centroid_x /= vertices.size();
    centroid_y /= vertices.size();

    // 7. Calculate the distance of each point from the centroid in the XY plane
    std::vector<double> distances;
    distances.reserve(vertices.size());
    for (const auto& v : vertices) {
        double dx = v[0] - centroid_x;
        double dy = v[1] - centroid_y;
        distances.push_back(std::sqrt(dx * dx + dy * dy));
    }

    // 8. Sort the distances
    std::sort(distances.begin(), distances.end());
    
    // 9. Trim outliers (e.g., the top 5% of largest distances)
    const double diameter_trim_percentage = 0.05; // 5% trim for diameter is often reasonable
    size_t diameter_trim_index = static_cast<size_t>(distances.size() * (1.0 - diameter_trim_percentage));
    
    // Ensure the index is valid
    if (diameter_trim_index >= distances.size()) {
        diameter_trim_index = distances.size() - 1;
    }
    
    // 10. The radius is the distance at our trimmed index. The diameter is twice that.
    double radius = distances[diameter_trim_index];
    double diameter = radius * 2.0;


    RCLCPP_INFO(LOGGER, "Calculated robust dimensions from '%s' -> Height: %.4f, Diameter: %.4f", ply_filepath.c_str(), height, diameter);
    return { height, diameter };
}

// 这个函数保持不变
moveit_msgs::msg::CollisionObject createCylinderObject(const std::string& object_id,
                                                       const std::string& frame_id,
                                                       const geometry_msgs::msg::Pose& pose,
                                                       double height,
                                                       double radius) {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = frame_id;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { height, radius };
    
    geometry_msgs::msg::Pose object_pose = pose;
    object_pose.position.z += 0.5 * height;
    object.primitive_poses.push_back(object_pose);
    return object;
}

} // namespace utils
} // namespace mtc_dynamic