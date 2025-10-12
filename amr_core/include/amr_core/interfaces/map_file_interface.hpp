#pragma once

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

/**
 * @brief Publishes map grid created by the robot and forbidden areas as MarkerArray for RViz 
 * visualization.
 */
class MapFileInterface
{
public:
  /**
   * @brief Constructor. Initializes publishers, loads map data, and starts timer.
   */
  MapFileInterface(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
  }

  /**
   * @brief Initializes the occupancy grid and forbidden areas.
   */
  void initialize()
  {
    map_pub_frequency_ = node_->declare_parameter<int>("map_data.frequency", 2);
    map_file_ = node_->declare_parameter<std::string>("map_data.file", "data.map");
    map_topic_ = node_->declare_parameter<std::string>("map_data.topic", "map_grid");

    map_pub_interval_ms_ = int(1000 / map_pub_frequency_);

    map_pub = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);
    fa_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("forbidden_areas", 10);
    timer = node_->create_wall_timer(std::chrono::milliseconds(map_pub_interval_ms_),
                                     std::bind(&MapFileInterface::publish_map, this));

    set_forbidden_area_params();

    if (!get_map_data(map_file_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Reading map failed");
    }

    get_min_max_coordinates();
    set_grid_attributes();
    initialize_map();
    fill_map();

    RCLCPP_INFO(node_->get_logger(), "MapFileInterface initialized");
  }

  /**
   * @brief This function publishes the occupancy grid and forbidden areas to their respective topics.
   */
  void publish_map()
  {
    map_pub->publish(grid);
  }

  /**
   * @brief Creates a quaternion msg given only yaw.
   * @param yaw Yaw angle in radians.
   * @return Quaternion message.
   */
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion tfq;
    tfq.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion gmq;

    double roll = 0;
    double pitch = 0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    gmq.w = cy * cp * cr + sy * sp * sr;
    gmq.x = cy * cp * sr - sy * sp * cr;
    gmq.y = sy * cp * sr + cy * sp * cr;
    gmq.z = sy * cp * cr - cy * sp * sr;

    return gmq;
  }

  /**
   * @brief Extracts yaw from a quaternion message.
   * @param quaternion Quaternion message.
   * @return Yaw angle in radians.
   */
  double getYawFromQuaternionMsg(geometry_msgs::msg::Quaternion quaternion)
  {
    double varA = +2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y));
    double varB = +1.0 - 2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z));
    return std::atan2(varA, varB);
  }

private:
  /**
   * @brief Gets minimum and maximum coordinates from the points vector.
   */
  void get_min_max_coordinates()
  {
    minx = std::numeric_limits<double>::max();
    miny = std::numeric_limits<double>::max();
    maxx = -std::numeric_limits<double>::max();
    maxy = -std::numeric_limits<double>::max();
    for (const auto& pt : points)
    {
      if (pt.x < minx)
        minx = pt.x;
      if (pt.x > maxx)
        maxx = pt.x;
      if (pt.y < miny)
        miny = pt.y;
      if (pt.y > maxy)
        maxy = pt.y;
    }
  }

  /**
   * @brief Gets the number of points from a map file line.
   * @param line Line from map file.
   * @param ignore_head Unused header string.
   * @param num_of_points Output number of points.
   */
  void get_points_count(std::string* line, std::string* ignore_head, int* num_of_points)
  {
    if (line->find("NumPoints") != std::string::npos)
    {
      std::istringstream iss(*line);
      iss >> *ignore_head;
      iss >> *num_of_points;
    }
  }

  /**
   * @brief Gets the number of lines from a map file line.
   * @param line Line from map file.
   * @param ignore_head Unused header string.
   * @param num_of_lines Output number of lines.
   */
  void get_lines_count(std::string* line, std::string* ignore_head, int* num_of_lines)
  {
    if (line->find("NumLines") != std::string::npos)
    {
      std::istringstream iss(*line);
      iss >> *ignore_head;
      iss >> *num_of_lines;
    }
  }

  /**
   * @brief Adds a point to the points vector.
   * @param x X coordinate (mm).
   * @param y Y coordinate (mm).
   */
  void set_point(int x, int y)
  {
    geometry_msgs::msg::Point p;

    // Values are in millimeter.
    p.x = static_cast<double>(x) / 1000.0;
    p.y = static_cast<double>(y) / 1000.0;
    p.z = 0;
    points.push_back(p);
  }

  /**
   * @brief Adds a line (two points) to the lines vector.
   * @param x1 X1 coordinate (mm).
   * @param y1 Y1 coordinate (mm).
   * @param x2 X2 coordinate (mm).
   * @param y2 Y2 coordinate (mm).
   */
  void set_line(int x1, int y1, int x2, int y2)
  {
    geometry_msgs::msg::Point p1, p2;

    // Values are in millimeter.
    p1.x = static_cast<double>(x1) / 1000.0;
    p1.y = static_cast<double>(y1) / 1000.0;
    p1.z = 0;
    p2.x = static_cast<double>(x2) / 1000.0;
    p2.y = static_cast<double>(y2) / 1000.0;
    p2.z = 0;

    // Needs two consecutive points to form a line.
    lines.push_back(p1);
    lines.push_back(p2);
  }

  /**
   * @brief Reads forbidden area information from a map file line.
   * @param line Line from map file.
   * @param dt_theta Output orientation (radians).
   * @param x1 Output X1.
   * @param y1 Output Y1.
   * @param x2 Output X2.
   * @param y2 Output Y2.
   */
  void get_forbidden_area_information(std::string line, double* dt_theta, double* x1, double* y1, double* x2,
                                      double* y2)
  {
    std::istringstream iss(line);
    std::string dummy;

    // The first 8 pieces of that line are irrelevant except for the 5th piece which is the angle/orientation
    // of the forbidden area.
    for (int i = 0; i < 8; i++)
    {
      if (i == 4)
      {
        iss >> *dt_theta;
        *dt_theta *= 0.01745329;
      }
      else
      {
        iss >> dummy;
      }
    }
    if (!(iss >> *y1 >> *x1 >> *y2 >> *x2))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error reading forbidden area from file");
    }
  }

  /**
   * @brief Sets forbidden area marker and adds to marker array.
   * @param dt_theta Orientation (radians).
   * @param x1 X1.
   * @param y1 Y1.
   * @param x2 X2.
   * @param y2 Y2.
   */
  void set_forbidden_area_information(double dt_theta, double x1, double y1, double x2, double y2)
  {
    // Compute the actual forbidden area from the transformed area.
    // The raw info in the map file is a transformed coordinates of the actual forbidden area.
    // We transform back to the actual area.
    double cx = (x1 + x2) / 2000.0;  // Centre of transformed X coord.
    double cy = (y1 + y2) / 2000.0;  // Centre of transformed Y coord.

    // Distance from centre of transformed area to origin.
    double r = sqrt((cx * cx) + (cy * cy));  // Polar coordinates, r component.
    double theta = atan2(cy, cx);            // Polar coordinates, theta component.

    // Get back the centre point of actual forbidden area.
    double o_theta = theta - dt_theta;
    double ocx = r * sin(o_theta);
    double ocy = r * cos(o_theta);
    dt_theta -= M_PI_2;

    geometry_msgs::msg::Quaternion q = createQuaternionMsgFromYaw(dt_theta);

    // Push to vector that will be published.
    fa.pose.position.x = ocx;
    fa.pose.position.y = ocy;
    fa.pose.orientation = q;
    fa.scale.x = (x2 - x1) / 1000.0;
    fa.scale.y = (y2 - y1) / 1000.0;
    f_areas.markers.push_back(fa);
    fa.id += 1;
  }

  /**
   * @brief Sets default parameters for forbidden area markers.
   */
  void set_forbidden_area_params()
  {
    fa.header.frame_id = "pose";
    fa.action = visualization_msgs::msg::Marker::ADD;
    fa.ns = "forbidden_areas";
    fa.pose.orientation.w = 1.0;
    fa.pose.position.z = 0;
    fa.type = visualization_msgs::msg::Marker::CUBE;
    fa.id = 0;
    fa.scale.z = 0.25;
    fa.color.a = 0.5;
    fa.color.r = 1.0;
    fa.color.g = 0.0;
    fa.color.b = 0.0;
  }

  /**
   * @brief Loads map data from file and populates points, lines, and forbidden areas.
   * @param filename Map file name.
   * @return True if successful, false otherwise.
   */
  bool get_map_data(std::string filename)
  {
    // Read map file
    std::string map_path = ament_index_cpp::get_package_share_directory("amr_ros") + "/map/" + filename;
    std::ifstream map_file(map_path);

    if (map_file.fail())
    {
      return false;
    }

    int num_of_points = 0;
    int num_of_lines = 0;
    std::string line, ignore_head;
    while (std::getline(map_file, line))
    {
      get_points_count(&line, &ignore_head, &num_of_points);
      get_lines_count(&line, &ignore_head, &num_of_lines);

      // We have seen the points data, collect them.
      if (line.find("DATA") != std::string::npos)
      {
        int x, y;
        for (int i = 0; i < num_of_points; i++)
        {
          std::getline(map_file, line);
          std::istringstream iss(line);
          if (!(iss >> x >> y))
          {
            RCLCPP_ERROR(node_->get_logger(), "Error reading points from file.");
          }
          set_point(x, y);
        }
      }

      // We have seen the lines data, collect them.
      if (line.find("LINES") != std::string::npos)
      {
        int x1, y1, x2, y2;
        for (int i = 0; i < num_of_lines; i++)
        {
          std::getline(map_file, line);
          std::istringstream iss(line);
          if (!(iss >> x1 >> y1 >> x2 >> y2))
          {
            RCLCPP_ERROR(node_->get_logger(), "Error reading lines from file.");
          }
          set_line(x1, y1, x2, y2);
        }
      }

      // node_ line contains forbidden area information, collect them.
      if (line.find("Cairn: ForbiddenArea") != std::string::npos)
      {
        // Read the raw info
        double x1, y1, x2, y2, dt_theta;
        get_forbidden_area_information(line, &dt_theta, &x1, &y1, &x2, &y2);
        set_forbidden_area_information(dt_theta, x1, y1, x2, x2);
      }
    }
    return true;
  }

  /**
   * @brief Gets the normal angle for calculating width. Assumes -180 to +180 degrees from 0 heading.
   * @param theta Input angle in radians.
   * @return Normal angle in radians.
   */
  double get_normal(double theta)
  {
    double pi = 3.141592654;
    if (theta >= pi / 2)
      theta -= pi / 2;
    else
      theta += pi / 2;
    return theta;
  }

  /**
   * @brief Returns coordinate difference for a given heading and distance.
   * @param theta Heading in radians.
   * @param distance Distance.
   * @return Point difference.
   */
  geometry_msgs::msg::Point get_extrapolated_coordinates(double theta, double distance)
  {
    // sin and cos functions take in radians
    double pi = 3.141592654;
    geometry_msgs::msg::Point difference;

    // 0 to 90 degrees
    if (theta >= 0 && theta <= (pi / 2))
    {
      difference.x = distance * cos(theta);
      difference.y = distance * sin(theta);
    }
    // 90 to 180 degrees
    else if (theta > (pi / 2) && theta <= pi)
    {
      difference.x = distance * cos(pi - theta);
      difference.y = -(distance * sin(pi - theta));
    }
    // 0 to -90 degrees
    else if (theta < 0 && theta >= -(pi / 2))
    {
      difference.x = -(distance * cos(theta));
      difference.y = distance * sin(theta);
    }
    // -90 to -180 degrees
    else if (theta < -(pi / 2) && theta >= -pi)
    {
      // assumes that theta is a negative value
      difference.x = -(distance * cos(theta + pi));
      difference.y = -(distance * sin(theta + pi));
    }
    return difference;
  }

  /**
   * @brief Checks if a point is within the map array and colours it.
   * @param coordinate Point coordinate.
   * @param occupancy_value Value to set in grid.
   */
  void check_and_colour_grid(geometry_msgs::msg::Point coordinate, int occupancy_value)
  {
    float relx = coordinate.x - minx;
    float rely = coordinate.y - miny;
    rely /= grid.info.resolution;
    relx /= grid.info.resolution;

    // Check if point is within map bounds
    if (rely <= grid.info.height && relx <= grid.info.width && rely >= 0 && relx >= 0)
    {
      // node_ is 2D array represented by 1D. correct position would be (height_position * width) + width
      int position_on_grid = int((int(rely) * grid.info.width) + relx);
      grid.data[position_on_grid] = occupancy_value;
    }
  }

  /**
   * @brief Fills the occupancy grid with points, lines, and forbidden areas.
   */
  void fill_map()
  {
    // For each forbidden area, get number of steps on each axis of rectangle
    for (size_t i = 0; i < f_areas.markers.size(); i++)
    {
      // Get number of steps to calculate to cover the full length and breadth of forbidden area
      int step_count_length = (f_areas.markers[i].scale.x / grid.info.resolution) + 1;
      int step_count_breadth = (f_areas.markers[i].scale.y / grid.info.resolution) + 1;

      // not sure if node_ should be radians or degree.
      double yaw = getYawFromQuaternionMsg(f_areas.markers[i].pose.orientation);

      // Get the x and y difference in step for length of forbidden area
      // The resolution here is divided by 2 to increase fidelity of the forbidden area
      auto step_length = get_extrapolated_coordinates(yaw, grid.info.resolution / 2);

      // Get the x and y difference in step for breadh of forbidden area. The angle node_ is trigonometrically
      // calculated from is the normal of the yaw of the original orientation
      auto step_breadth = get_extrapolated_coordinates(get_normal(yaw), grid.info.resolution / 2);

      // Since we have an origin point, begin filling rectangle from origin.
      // For each row in the forbidden area, get their starting coordinates
      for (int j = 0; j < step_count_breadth; j++)
      {
        geometry_msgs::msg::Point starting_position_positive, starting_position_negative;

        starting_position_positive.x = f_areas.markers[i].pose.position.x + (step_breadth.x * j);
        starting_position_positive.y = f_areas.markers[i].pose.position.y + (step_breadth.y * j);

        starting_position_negative.x = f_areas.markers[i].pose.position.x - (step_breadth.x * j);
        starting_position_negative.y = f_areas.markers[i].pose.position.y - (step_breadth.y * j);

        // get the position of every point in a row to check and colour the grid
        for (int k = 0; k < step_count_length; k++)
        {
          // Occupancy value here is set to 85 just to distinguish between forbidden areas and actual map obstacles
          // They should be set to 100 if map is used for path planning (We are only using it for visualization)
          geometry_msgs::msg::Point starting_position_positive_positive;
          starting_position_positive_positive.x = starting_position_positive.x + (step_length.x * k);
          starting_position_positive_positive.y = starting_position_positive.y + (step_length.y * k);
          check_and_colour_grid(starting_position_positive_positive, 85);

          geometry_msgs::msg::Point starting_position_positive_negative;
          starting_position_positive_negative.x = starting_position_positive.x - (step_length.x * k);
          starting_position_positive_negative.y = starting_position_positive.y - (step_length.y * k);
          check_and_colour_grid(starting_position_positive_negative, 85);

          geometry_msgs::msg::Point starting_position_negative_positive;
          starting_position_negative_positive.x = starting_position_negative.x + (step_length.x * k);
          starting_position_negative_positive.y = starting_position_negative.y + (step_length.y * k);
          check_and_colour_grid(starting_position_negative_positive, 85);

          geometry_msgs::msg::Point starting_position_negative_negative;
          starting_position_negative_negative.x = starting_position_negative.x - (step_length.x * k);
          starting_position_negative_negative.y = starting_position_negative.y - (step_length.y * k);
          check_and_colour_grid(starting_position_negative_negative, 85);
        }
      }
    }

    // for each point in point vector, convert to relative coordinates compared to grid origin
    // if point lies within a grid, colour it black
    for (const auto& pt : points)
    {
      check_and_colour_grid(pt, 100);
    }

    for (size_t i = 0; i < lines.size(); i++)
    {
      // for each 2 points in line vector, convert to relative coordinates compared to grid origin
      // break each line up into as many points as grid resolution allows
      // check if points lie within a grid and colour it black
      if (i % 2 == 1)
        continue;

      // gets vector from 1 point to another
      double vector_x = lines[i + 1].x - lines[i].x;
      double vector_y = lines[i + 1].y - lines[i].y;

      // calculates magnitude of the vector
      double distance = std::hypot(vector_x, vector_y);

      // break the distance into points based on grid resolution
      int steps = distance / grid.info.resolution;
      float step_x = vector_x / steps;
      float step_y = vector_y / steps;

      // incrementally checks each point from the start of the line where they lie on the grid
      for (int j = 0; j < steps; j++)
      {
        geometry_msgs::msg::Point step_temp;
        step_temp.x = lines[i].x + (step_x * j);
        step_temp.y = lines[i].y + (step_y * j);
        check_and_colour_grid(step_temp, 100);
      }
    }
  }

  /**
   * @brief Sets entire OccupancyGrid to -1 for unknown value in grid.
   */
  void initialize_map()
  {
    for (int i = 0; i < int(grid.info.height * grid.info.width); i++)
    {
      grid.data.push_back(-1);
    }
  }

  /**
   * @brief Sets the necessary attributes in the occupancy grid to show up in RViz.
   */
  void set_grid_attributes()
  {
    grid.header.stamp = node_->get_clock()->now();
    grid.info.resolution = 0.02;
    grid.info.origin.orientation = createQuaternionMsgFromYaw(0);
    grid.header.frame_id = "map";

    geometry_msgs::msg::Point origin;
    origin.x = minx;
    origin.y = miny;
    origin.z = 0;
    grid.info.origin.position = origin;

    grid.info.width = int((maxx - minx) / grid.info.resolution + 1);
    grid.info.height = int((maxy - miny) / grid.info.resolution + 1);
  }

  std::vector<geometry_msgs::msg::Point> points;  ///< Map points
  std::vector<geometry_msgs::msg::Point> lines;   ///< Map lines
  visualization_msgs::msg::MarkerArray f_areas;   ///< Forbidden areas marker array
  visualization_msgs::msg::Marker fa;             ///< Single forbidden area marker
  nav_msgs::msg::OccupancyGrid grid;              ///< Occupancy grid
  double minx, miny, maxx, maxy;                  ///< Map bounds

  rclcpp::Node::SharedPtr node_;  ///< Node pointer
  std::string map_file_;          ///< Map file name
  std::string map_topic_;         ///< Map Topic

  int map_pub_frequency_{ 2 };
  int map_pub_interval_ms_{ 500 };

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;         ///< Publisher for occupancy grid
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fa_pub;  ///< Publisher for forbidden areas
  rclcpp::TimerBase::SharedPtr timer;                                         ///< Timer for periodic publishing
};