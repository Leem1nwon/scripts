config = [
(0, 7, {
    'base_speed': 42,
    'speed_reduction_factor': 0.9,
    'lfd_base': 5.5,
    'raw_steering_divider': 5.5,
    'weight_factor' : 0.7,
    'steering_threshold' : 0.01
}),
(7, 45, {
    'base_speed': 42,
    'speed_reduction_factor': 0.88,
    'lfd_base': 5.5,
    'raw_steering_divider': 5.8,
    'weight_factor' : 0.85,
    'steering_threshold' : 0.08
}),
(45, 52, {
    'base_speed': 38,
    'speed_reduction_factor': 0.9,
    'lfd_base': 6.0,
    'raw_steering_divider': 7.2,
    'weight_factor' : 0.85,
    'steering_threshold' : 0.08
}),
(52, 59, {
    'base_speed': 23,
    'speed_reduction_factor': 0.9,
    'lfd_base': 4.5,
    'raw_steering_divider': 5.0,
    'weight_factor' : 0.75,
    'steering_threshold' : 0.09
}),
(59, 86, {   # first left curve
    'base_speed': 27,
    'speed_reduction_factor': 0.9,
    'lfd_base': 2.7,
    'raw_steering_divider': 1.6,
    'weight_factor' : 0.22,
    'steering_threshold' : 0.09
}),
(86, 92, {
    'base_speed': 35,
    'speed_reduction_factor': 0.85,
    'lfd_base': 5.0,
    'raw_steering_divider': 4.0,
    'weight_factor' : 0.8,
    'steering_threshold' : 0.09
}),
(92, 107, { #second curve
    'base_speed': 30,
    'speed_reduction_factor': 0.9,
    'lfd_base': 2.5,
    'raw_steering_divider': 1.6,
    'weight_factor' : 0.22,
    'steering_threshold' : 0.09
}),
(107, 122, {
    'base_speed': 42,
    'speed_reduction_factor': 0.8,
    'lfd_base': 4.0,
    'raw_steering_divider': 4.5,
    'weight_factor' : 0.8,
    'steering_threshold' : 0.09
}),
(122, 142, {#신호등1코너
    'base_speed': 35,
    'speed_reduction_factor': 0.9,
    'lfd_base': 2.0,
    'raw_steering_divider': 1.7,
    'weight_factor' : 0.22,
    'steering_threshold' : 0.09
}),
(142, 219, { #s자코너
    'base_speed': 35,
    'speed_reduction_factor': 0.9,
    'lfd_base': 1.7,
    'raw_steering_divider': 1.7,
    'weight_factor' : 0.28,
    'steering_threshold' : 0.09
}),
(219, 250, {
    'base_speed': 42,
    'speed_reduction_factor': 0.8,
    'lfd_base': 5.5,
    'raw_steering_divider': 5.3,
    'weight_factor' : 0.85,
    'steering_threshold' : 0.09
}),
(250, 263, {
    'base_speed': 35,
    'speed_reduction_factor': 0.9,
    'lfd_base': 5.0,
    'raw_steering_divider': 4.6,
    'weight_factor' : 0.85,
    'steering_threshold' : 0.09
}),
(263, 298, {
    'base_speed': 30,
    'speed_reduction_factor': 0.9,
    'lfd_base': 2.2,
    'raw_steering_divider': 1.5,
    'weight_factor' : 0.22,
    'steering_threshold' : 0.09
}),
(298, 310, {
    'base_speed': 45,
    'speed_reduction_factor': 0.9,
    'lfd_base': 4.0,
    'raw_steering_divider': 4.0,
    'weight_factor' : 0.75,
    'steering_threshold' : 0.09
}), 
(310, 356, {
    'base_speed': 28,
    'speed_reduction_factor': 0.9,
    'lfd_base': 2.5,
    'raw_steering_divider': 1.7,
    'weight_factor' : 0.21,
    'steering_threshold' : 0.09
}),
(356, 374, {
    'base_speed': 45,
    'speed_reduction_factor': 0.85,
    'lfd_base': 4.0,
    'raw_steering_divider': 4.0,
    'weight_factor' : 0.75,
    'steering_threshold' : 0.09
}),
(374, 397, {
    'base_speed': 28,
    'speed_reduction_factor': 0.9,
    'lfd_base': 2.5,
    'raw_steering_divider': 1.7,
    'weight_factor' : 0.28,
    'steering_threshold' : 0.09
}),
(397, 400, {
    'base_speed': 0.0,
    'speed_reduction_factor': 0.9,
    'lfd_base': 5.0,
    'raw_steering_divider': 2,
    'weight_factor' : 0.75,
    'steering_threshold' : 0.09
}), 
]