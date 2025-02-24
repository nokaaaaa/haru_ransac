#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>
#include <exception>

using namespace std;
using namespace std::placeholders;

//でかい変更 角の隅を(0,0)にして,フィールドの寸法から自己位置を決定する 目を覚まさせてくれた川合さんに感謝

class RANSAC_Euclid {
public:
    RANSAC_Euclid(int max_loop, double threshold, int min_samples)
        : max_loop_(max_loop), threshold_(threshold), min_samples_(min_samples) {}

    vector<double> get_params(const pair<vector<double>, vector<double>> &samples) {
        double x1 = samples.first[0], y1 = samples.first[1];
        double x2 = samples.second[0], y2 = samples.second[1];
        double a = y1 - y2;
        double b = x2 - x1;
        double c = x1 * y2 - x2 * y1;
        return {a, b, c};
    }

    double euclid(const vector<double> &params, const vector<double> &p) {
        double a = params[0], b = params[1], c = params[2];
        double x = p[0], y = p[1];
        return abs(a * x + b * y + c) / sqrt(a * a + b * b);
    }

    tuple<vector<double>, vector<bool>> run_ransac(const vector<vector<double>> &data) {
        int iterations = 0;
        vector<vector<double>> models_params;
        vector<double> model_errors;
        vector<vector<bool>> inlier_masks;

        while (iterations < max_loop_) {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> dis(0, data.size() - 1);

            int idx1 = dis(gen), idx2 = dis(gen);
            if (idx1 == idx2) continue;

            auto params = get_params({data[idx1], data[idx2]});
            vector<bool> inlier_mask(data.size(), false);
            int inlier_count = 0;

            for (size_t i = 0; i < data.size(); ++i) {
                if (euclid(params, data[i]) <= threshold_) {
                    inlier_mask[i] = true;
                    ++inlier_count;
                }
            }

            if (inlier_count >= min_samples_) {
                double error = 0.0;
                for (size_t i = 0; i < data.size(); ++i) {
                    if (inlier_mask[i]) error += euclid(params, data[i]);
                }
                error /= inlier_count;

                models_params.push_back(params);
                model_errors.push_back(error);
                inlier_masks.push_back(inlier_mask);
            }
            ++iterations;
        }

        if (!model_errors.empty()) {
            auto min_error_it = min_element(model_errors.begin(), model_errors.end());
            size_t best_index = distance(model_errors.begin(), min_error_it);
            return {models_params[best_index], inlier_masks[best_index]};
        }

        throw runtime_error("No valid line found.");
    }

private:
    int max_loop_;
    double threshold_;
    int min_samples_;
};

class RANSACNode : public rclcpp::Node {
public:
    RANSACNode() : Node("hransac") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_fullframe", 10, bind(&RANSACNode::lidar_callback, this, _1));
        state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/state", 10,
            bind(&RANSACNode::state_callback, this, std::placeholders::_1));

        line_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/line_markers", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);


        this->declare_parameter<int>("max_loop", 100);
        this->declare_parameter<double>("threshold", 0.05);
        this->declare_parameter<int>("min_samples", 10);
        this->declare_parameter<int>("detect_line", 5);
        this->declare_parameter<double>("min_theta", -90.0);
        this->declare_parameter<double>("max_theta", 90.0);
        this->declare_parameter<int>("sampling_rate", 2);
        this->declare_parameter<double>("tf_y", 0.237673);
        this->declare_parameter<bool>("visu_line", true);
        this->declare_parameter("field_color", "red");


        max_loop_ = this->get_parameter("max_loop").as_int();
        threshold_  = this->get_parameter("threshold").as_double();
        min_samples_ = this->get_parameter("min_samples").as_int();
        detect_line_ = this->get_parameter("detect_line").as_int();
        min_theta_ = this->get_parameter("min_theta").as_double();
        max_theta_  = this->get_parameter("max_theta").as_double();
        sampling_rate_ = this->get_parameter("sampling_rate").as_int();
        tf_y  = this->get_parameter("tf_y").as_double();
        visu_line_  = this->get_parameter("visu_line").as_bool();
        field_color_ = this->get_parameter("field_color").as_string();

        if(field_color_ == "red") court = true;
        else court = false;
        
       // RCLCPP_INFO(this->get_logger(), "field_color: %s", field_color_.c_str());

        cout<<"sampling_rate"   <<sampling_rate_<<endl;
    }

private:

    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    
    int max_loop_, min_samples_, detect_line_, sampling_rate_;
    double threshold_, min_theta_, max_theta_;
    double wid , hei ;
    double tf_y;
    bool visu_line_,court;
    string field_color_;

    size_t previous_marker_count_ = 0;
    int state = 0;


    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        vector<vector<double>> point_cloud;

        //注意! x座標とy座標が逆になっている

        //最初の経路  angleが30度以下の点群を取る 3本検出して、うち2本を使って自己位置決定

        //黒板消し→ボールの経路 xが0.5以下の点を選んで、5辺読んで2本選んで自己位置決定

        //細道に侵入する経路 and 射出時のyaw角参照　angleが-10度以上の点を選んで、3辺読んで2本選んで自己位置決定

        //↑これはあくまでLiDARが反転していない青コートを想定している
        //セグフォに気をつけよう×n

        for (size_t i = 0; i < msg->ranges.size(); i += sampling_rate_) {
            double range = msg->ranges[i];
            double angle = msg->angle_min + i * msg->angle_increment;

            if (range >= msg->range_min && range <= msg->range_max &&
                angle >= deg_to_rad(min_theta_) && angle <= deg_to_rad(max_theta_)) {
                double x = range * cos(angle);
                double y = range * sin(angle);

                if(court)
                {
                    if (state==0 &&angle<deg_to_rad(30)) 
                    {
                        detect_line_ = 3;
                        point_cloud.emplace_back(vector<double>{x, y});
                    }
                    if (state==1 &&x<0.6) 
                    {
                        detect_line_ = 5;
                        point_cloud.emplace_back(vector<double>{x, y});
                    }
                    if (state>=2 &&angle>deg_to_rad(-10)) 
                    {
                        detect_line_ = 3;
                        point_cloud.emplace_back(vector<double>{x, y});
                    }
                }
                else
                {                
                    if (state==0 &&angle>deg_to_rad(-30)) 
                    {
                        detect_line_ = 3;
                        point_cloud.emplace_back(vector<double>{x, y});
                    }
                    if (state==1 &&x<0.6) 
                    {
                        detect_line_ = 5;
                        point_cloud.emplace_back(vector<double>{x, y});
                    }
                    if (state>=2 &&angle<deg_to_rad(10)) 
                    {
                        detect_line_ = 3;
                        point_cloud.emplace_back(vector<double>{x, y});
                    }
                }
                
            }

        }

        vector<vector<double>> params;
        vector<vector<vector<double>>> inlier_points_list;

        for (int i = 0; i < detect_line_; ++i) {
            if (point_cloud.size() < 2) break;

            RANSAC_Euclid ransac(max_loop_, threshold_, min_samples_);
            try {
                auto [best_params, inlier_mask] = ransac.run_ransac(point_cloud);
                params.push_back(best_params);

                vector<vector<double>> inlier_points;
                vector<vector<double>> new_point_cloud;
                for (size_t j = 0; j < point_cloud.size(); ++j) {
                    if (inlier_mask[j]) {
                        inlier_points.push_back(point_cloud[j]);
                    } else {
                        new_point_cloud.push_back(point_cloud[j]);
                    }
                }
                inlier_points_list.push_back(inlier_points);
                point_cloud = new_point_cloud;
            } catch (const exception &e) {
                RCLCPP_INFO(this->get_logger(), "RANSAC could not find a line: %s", e.what());
                break;
            }
        }
       
        if(visu_line_) line_visualizer(params, inlier_points_list);

        //検出した直線の本数
        int line_num = params.size();
        //直線の係数 ax+by+c=0
        vector<double> a(line_num), b(line_num), c(line_num);
        for (int i = 0; i < line_num; ++i) {
            //係数の正規化(aが常に正)
            if(params[i][0]<0)
            {
                a[i] = -1*params[i][0]; 
                b[i] = -1*params[i][1]; 
                c[i] = -1*params[i][2]; 
            }
            else
            {
                a[i] = params[i][0]; 
                b[i] = params[i][1]; 
                c[i] = params[i][2];
            }
        }
        
         //一番傾きの絶対値が大きい直線を探す
        int index = 0;
        for(int i=0;i<line_num;++i)
        {
            if(abs(a[i]/b[i])>abs(a[index]/b[index])) index = i;
        }

        //indexを除き、ローカル座標のy軸との交点が最大の直線を選ぶ
        int max_index = -1;
        double max_value = std::numeric_limits<double>::min();

        for (int i = 0; i < line_num; ++i) {
            if (i == index) continue; 
            double hvalue = -c[i] / b[i];
            if (hvalue > max_value) {
                max_value = hvalue;
                max_index = i;
            }
        }

        //indexを除き、ローカル座標のy軸との交点が最小の直線を選ぶ
        int min_index = -1;
            double min_value = std::numeric_limits<double>::max();

            for (int i = 0; i < line_num; ++i) {
                if (i == index) continue;
                double lvalue = -c[i] / b[i];
                if (lvalue < min_value) {
                    min_value = lvalue;
                    min_index = i;
                }
            }

        double pose_x , pose_y , pose_theta ;

        if(state==0 && court == true)
        {
            pose_theta = normalize_angle(-atan(a[min_index]/b[min_index]));
            pose_x = (-1) * tf_y * sin(pose_theta) - distance(a[min_index],b[min_index],c[min_index]);
            pose_y = tf_y * cos(pose_theta) + distance(a[index],b[index],c[index]);
        }
        if(state==0 && court == false)
        {
            pose_theta = normalize_angle(-atan(a[max_index]/b[max_index]));
            pose_x = (-1) * tf_y * sin(pose_theta) + distance(a[max_index],b[max_index],c[max_index]);
            pose_y = tf_y * cos(pose_theta) + distance(a[index],b[index],c[index]);
        }
        if(state==1 && court == true)
        { 
            pose_theta = normalize_angle(-atan(a[max_index]/b[max_index]));
            pose_x = (-1) * tf_y * sin(pose_theta) + distance(a[max_index],b[max_index],c[max_index])-3.424;
            pose_y = tf_y * cos(pose_theta) + distance(a[index],b[index],c[index])+5.5;
        }
        if(state==1 && court == false)
        {
            pose_theta = normalize_angle(-atan(a[min_index]/b[min_index]));
            pose_x = (-1) * tf_y * sin(pose_theta) - distance(a[min_index],b[min_index],c[min_index])+3.424;
            pose_y = tf_y * cos(pose_theta) + distance(a[index],b[index],c[index])+5.5;
        }
        if(state>=2 && court == true)
        {
            pose_theta = normalize_angle(-atan(a[max_index]/b[max_index]));
            pose_x = (-1) * tf_y * sin(pose_theta) + distance(a[max_index],b[max_index],c[max_index])-3.424;
            pose_y = tf_y * cos(pose_theta) + distance(a[index],b[index],c[index])+2.962;
        }
        if(state>=2 && court == false)
        {
            pose_theta = normalize_angle(-atan(a[min_index]/b[min_index]));
            pose_x = (-1) * tf_y * sin(pose_theta) - distance(a[min_index],b[min_index],c[min_index])+3.424;
            pose_y = tf_y * cos(pose_theta) + distance(a[index],b[index],c[index])+2.962;
        }

        geometry_msgs::msg::Pose2D pose_msg;
        pose_msg.x = pose_x;
        pose_msg.y = pose_y;
        pose_msg.theta = pose_theta;
        pose_publisher_->publish(pose_msg);
      
    }

    void state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        state = msg->data;
    }
    
    //原点と直線の距離
    double distance(double s, double t, double u) {
        return abs(u)/sqrt(s*s+t*t);
    }

    void line_visualizer(const vector<vector<double>> &params, const vector<vector<vector<double>>> &inlier_points_list) {
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        for (size_t i = 0; i < params.size(); ++i) {
            const auto &param = params[i];
            const auto &inlier_points = inlier_points_list[i];
            double a = param[0], b = param[1], c = param[2];

            double x_min = numeric_limits<double>::max();
            double x_max = numeric_limits<double>::lowest();
            double y_min = numeric_limits<double>::max();
            double y_max = numeric_limits<double>::lowest();

            for (const auto &point : inlier_points) {
                x_min = min(x_min, point[0]);
                x_max = max(x_max, point[0]);
                y_min = min(y_min, point[1]);
                y_max = max(y_max, point[1]);
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "lines";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.05;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            geometry_msgs::msg::Point p1, p2;

            const double epsilon = 1e-6;
            if (abs(b) > epsilon) {
                double y1 = (-c - a * x_min) / b;
                double y2 = (-c - a * x_max) / b;
                p1.x = x_min;
                p1.y = y1;
                p1.z = 0.0;
                p2.x = x_max;
                p2.y = y2;
                p2.z = 0.0;
            } else {
                double x_const = -c / a;
                p1.x = x_const;
                p1.y = y_min - 1.0;
                p1.z = 0.0;
                p2.x = x_const;
                p2.y = y_max + 1.0;
                p2.z = 0.0;
            }

            marker.points.push_back(p1);
            marker.points.push_back(p2);

            marker_array.markers.push_back(marker);
        }

        if (marker_array.markers.size() < previous_marker_count_) {
            for (size_t i = marker_array.markers.size(); i < previous_marker_count_; ++i) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "lines";
                marker.id = i;
                marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_array.markers.push_back(marker);
            }
        }

        previous_marker_count_ = marker_array.markers.size();
        line_publisher_->publish(marker_array);
    }

    double deg_to_rad(double deg) {
        return deg * M_PI / 180.0;
    }
    double rad_to_deg(double rad) {
        return rad * 180.0 / M_PI;
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<RANSACNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
