extends RayCast3D


@export_category("RayCast Wheel")
@export var use_as_traction := false
@export var use_as_steering := false
@export var max_steering_angle := 35.0
@export_group("Spring Properties")
@export var spring_rest_length := 0.7
@export var spring_stiffness := 70.0
@export var spring_damper := 4.0
@export_group("Tire Properties")
@export var tire_radius := 0.34
@export var tire_mass := 1.0
@export_range(0.0, 1.0, 0.01) var tire_grip_factor := 0.01


var steering_speed := 50.0
var steering_input := 0.0
var throttle_input := 0.0
var spring_curr_length := 0.0
var curr_steering_angle := 0.0
var tire_world_velocity := Vector3.ZERO

var car_body: RigidBody3D
var wheel_mesh: Node3D


func _ready() -> void:
	car_body = $".." as RigidBody3D
	wheel_mesh = $WheelMesh as Node3D
	
	# in Godot 3.x this property is called `cast_to`
	target_position = Vector3.DOWN * spring_rest_length
	
	spring_curr_length = spring_rest_length
	spring_curr_length = clampf(spring_curr_length, 0.0, spring_rest_length)


func _process(delta: float) -> void:
	steering_input = Input.get_axis("ui_right", "ui_left")
	throttle_input = Input.get_axis("ui_down", "ui_up")
	
	if use_as_steering:
		if is_equal_approx(steering_input, 0.0):
			curr_steering_angle = move_toward(curr_steering_angle, 0.0, delta * steering_speed * 2.0)
		else:
			var target_steering_angle := max_steering_angle * steering_input
			curr_steering_angle = move_toward(curr_steering_angle, target_steering_angle, delta * steering_speed)
	
	rotation_degrees = Vector3.UP * curr_steering_angle
	
	wheel_mesh.position = Vector3.DOWN * (spring_curr_length - tire_radius)
	wheel_mesh.rotate_x(global_transform.basis.z.dot(tire_world_velocity) * delta / tire_radius) 


func _physics_process(delta: float) -> void:
	tire_world_velocity = get_point_velocity(global_position)
	
	if is_colliding():
		var force_point := get_collision_point() - car_body.global_position
		
		car_body.apply_force(get_spring_force(), force_point)
		car_body.apply_force(get_steering_force(delta), force_point)
		
		if use_as_traction:
			car_body.apply_force(get_throttle_force(), force_point)


func get_spring_force() -> Vector3:
	var spring_dir := get_collision_normal()
	
	spring_curr_length = get_collision_point().distance_to(global_position)
	spring_curr_length = clampf(spring_curr_length, 0.0, spring_rest_length)
	
	var spring_offset := spring_rest_length - spring_curr_length
	var spring_velocity := spring_dir.dot(tire_world_velocity)
	var spring_force := (spring_offset * spring_stiffness) - (spring_velocity * spring_damper)
	
	return spring_dir * spring_force


func get_steering_force(delta: float) -> Vector3:
	var steering_dir := global_transform.basis.x
	
	var steering_velocity := steering_dir.dot(tire_world_velocity)
	var desired_velocity_change := -steering_velocity * tire_grip_factor
	var desired_acceleration := desired_velocity_change / delta
	
	return steering_dir * tire_mass * desired_acceleration


func get_throttle_force() -> Vector3:
	var acceleration_dir := -global_transform.basis.z
	
	return acceleration_dir * throttle_input


# prefix `gs_` stands for `global space`
func get_point_velocity(gs_point: Vector3) -> Vector3:
	var linear_velocity := car_body.linear_velocity
	var angular_velocity := car_body.angular_velocity
	var gs_position := car_body.global_position
	
	return linear_velocity + angular_velocity.cross(gs_point - gs_position)
