#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "PrecisionLandCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/target_pose", rclcpp::QoS(1).best_effort(),
		std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
		std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	loadParameters();
}

void PrecisionLand::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		auto tag = ArucoTag{
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		_tag = getTagWorld(tag);
	}
}

PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	     1,  0, 0,
	     0,  0, 1;
	Eigen::Quaterniond quat_NED(R);

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void PrecisionLand::onActivate()
{
	generateSearchWaypoints();
	_search_started = true;
	switchToState(State::Search);
}

void PrecisionLand::onDeactivate()
{
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	bool target_lost = checkTargetTimeout();

	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired");
	}

	_target_lost_prev = target_lost;

	switch (_state) {
	case State::Idle:
		break;

	case State::Search: {
		if (!std::isnan(_tag.position.x())) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		auto waypoint_position = _search_waypoints[_search_waypoint_index];
		_trajectory_setpoint->updatePosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

	case State::Approach: {
		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);
		_trajectory_setpoint->updatePosition(target_position);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {
		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		if (_vehicle_local_position->positionNed().z() >= -6.0) {
			RCLCPP_INFO(_node.get_logger(), "Abort Descend: Current altitude is <= 6 meters");
			ModeBase::completed(px4_ros2::Result::Success);
			switchToState(State::Finished);
			return;
		}

		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
					     px4_ros2::quaternionToYaw(_tag.orientation));

		if (_land_detected) {
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished:
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -max_integral, max_integral);

	float vx = -1.f * (delta_pos_x * p_gain + _vel_x_integral * i_gain);
	float vy = -1.f * (delta_pos_y * p_gain + _vel_y_integral * i_gain);

	vx = std::clamp(vx, -_param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -_param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

bool PrecisionLand::checkTargetTimeout()
{
	if (!_tag.valid()) {
		return true;
	}

	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void PrecisionLand::generateSearchWaypoints()
{
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	int num_layers = ((static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1)
		? 1
		: (static_cast<int>((min_z - current_z) / layer_spacing) / 2);

	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		current_z += layer_spacing;

		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();
	const auto delta_pos = target - position;

	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::Idle: return "Idle";
	case State::Search: return "Search";
	case State::Approach: return "Approach";
	case State::Descend: return "Descend";
	case State::Finished: return "Finished";
	default: return "Unknown";
	}
}

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
