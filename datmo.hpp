#pragma once
#ifndef DATMO_HPP
#define DATMO_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <fstream>
#include <string>
#include <vector>
#include <cmath>

// ===========================================================================
// CONSTANTES GÉNÉRALES
// ===========================================================================
inline constexpr float detection_threshold = 0.2f;
inline constexpr int   dynamic_threshold_  = 75;
inline constexpr float default_cluster_threshold = 0.05f; // 5 cm

// Tailles typiques de jambes et distances entre jambes
inline constexpr float leg_size_min_      = 0.05f;
inline constexpr float leg_size_max_      = 0.25f;
inline constexpr float legs_distance_min_ = 0.1f;
inline constexpr float legs_distance_max_ = 0.7f;

// Suivi
inline constexpr int   frequency_init = 5;
inline constexpr int   frequency_max  = 25;

inline constexpr float uncertainty_init = 0.5f;
inline constexpr float uncertainty_min  = 0.5f;
inline constexpr float uncertainty_max  = 1.0f;
inline constexpr float uncertainty_inc  = 0.05f;

inline constexpr float robair_size       = 0.25f;
inline constexpr float goal_point_offset = 0.3f;

// ===========================================================================
// CLASSE DATMO
// ===========================================================================
class Datmo : public rclcpp::Node
{
public:
  Datmo();

  // Boucle principale
  void update();
  void update_state_variables();

  // -------------------------------------------------------------------------
  // Détection de mouvement (background subtraction)
  // -------------------------------------------------------------------------
  void store_background();
  void reset_motion();
  void detect_current_motion();
  void detect_motion();

  // -------------------------------------------------------------------------
  // Clustering
  // -------------------------------------------------------------------------
  void perform_clustering(float clustering_threshold = default_cluster_threshold);
  void perform_basic_clustering(float clustering_threshold = default_cluster_threshold);
  void perform_advanced_clustering();
  int  compute_nb_dynamic(int start, int end);

  // -------------------------------------------------------------------------
  // Détection de jambes et personnes
  // -------------------------------------------------------------------------
  void detect_legs();
  void detect_persons();
  void detect_a_moving_person();

  // -------------------------------------------------------------------------
  // Suivi de personnes
  // -------------------------------------------------------------------------
  virtual void initialize_tracking();
  virtual void track_a_person();
  virtual void detect_and_track_a_person();

  // -------------------------------------------------------------------------
  // Autres fonctionnalités
  // -------------------------------------------------------------------------
  void find_closest_obstacle();

  // Callbacks ROS
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void robot_moving_callback(const std_msgs::msg::Bool::SharedPtr state);

  // Utilitaires
  float distancePoints(const geometry_msgs::msg::Point &pa, const geometry_msgs::msg::Point &pb);
  void  save_data();

  // -------------------------------------------------------------------------
  // Visualisation RViz
  // -------------------------------------------------------------------------
  void display_field_of_view();
  void display_motion();
  void display_clustering();
  void display_legs();
  void display_persons();
  void display_a_moving_detected_person();
  virtual void display_a_tracked_person();
  void display_a_detected_and_tracked_person();

protected:
  // -------------------------------------------------------------------------
  // Souscriptions ROS
  // -------------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         sub_robot_moving_;

  // -------------------------------------------------------------------------
  // Publishers pour visualisation et données
  // -------------------------------------------------------------------------
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_field_of_view_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_motion_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_clusters_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_legs_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_persons_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_moving_detected_person_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_tracked_person_marker_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr       pub_detection_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr       pub_tracking_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr       pub_closest_obstacle_;

  // -------------------------------------------------------------------------
  // Données Laser
  // -------------------------------------------------------------------------
  int nb_beams_ = 0;
  std::vector<float>                      ranges_; // valeurs des distances
  std::vector<geometry_msgs::msg::Point>  points_; // coordonnées des points (x,y,z)

  // -------------------------------------------------------------------------
  // Détection de mouvement
  // -------------------------------------------------------------------------
  std::vector<float>                     background_ranges_;        // fond sauvegardé (distances)
  std::vector<geometry_msgs::msg::Point> background_points_;        // fond sauvegardé (points)
  std::vector<bool>                      dynamic_;                  // vrai si le point est mobile
  bool current_robot_moving_  = false;
  bool previous_robot_moving_ = false;

  // -------------------------------------------------------------------------
  // Clustering
  // -------------------------------------------------------------------------
  int nb_clusters_ = 0;
  std::vector<int>                      cluster_start_;
  std::vector<int>                      cluster_end_;
  std::vector<float>                    cluster_size_;
  std::vector<geometry_msgs::msg::Point> cluster_middle_;
  std::vector<float>                    cluster_dynamic_;   // pourcentage de points dynamiques
  std::vector<int>                      cluster_nb_points_;

  // -------------------------------------------------------------------------
  // Détection de jambes
  // -------------------------------------------------------------------------
  int nb_legs_detected_ = 0;
  std::vector<int>  leg_cluster_;
  std::vector<bool> leg_detected_;
  std::vector<bool> leg_dynamic_;

  // -------------------------------------------------------------------------
  // Détection de personnes
  // -------------------------------------------------------------------------
  int nb_persons_detected_ = 0;
  std::vector<geometry_msgs::msg::Point> detected_person_;
  std::vector<bool>                      person_dynamic_;

  // -------------------------------------------------------------------------
  // Détection d'une personne en mouvement
  // -------------------------------------------------------------------------
  bool                     is_moving_person_detected_ = false;
  geometry_msgs::msg::Point moving_detected_person_;

  // -------------------------------------------------------------------------
  // Suivi d'une personne
  // -------------------------------------------------------------------------
  bool                     is_person_tracked_  = false;
  bool                     was_person_tracked_ = false;
  geometry_msgs::msg::Point tracked_person_;
  bool  associated_ = false;
  int   index_min_  = -1;
  int   frequency_  = frequency_init;
  float uncertainty_ = uncertainty_init;

  // -------------------------------------------------------------------------
  // Etats d'init/données fraîches (utilisés par les *nodes*)
  // -------------------------------------------------------------------------
  bool init_laser_ = false;
  bool new_laser_  = false;
  bool init_robot_ = false;
  bool new_robot_  = false;

  // -------------------------------------------------------------------------
  // Visualisation
  // -------------------------------------------------------------------------
  visualization_msgs::msg::Marker marker_field_of_view_;
  visualization_msgs::msg::Marker marker_motion_;
  visualization_msgs::msg::Marker marker_clusters_;
  visualization_msgs::msg::Marker marker_legs_;
  visualization_msgs::msg::Marker marker_persons_;
  visualization_msgs::msg::Marker marker_moving_detected_person_;
  visualization_msgs::msg::Marker marker_tracked_person_;
};

#endif // DATMO_HPP
