define laser ranger(
	sensor(
		range [0.0 5.6]
		fov 360.0
		samples 360
	)

	color "blue"
	size [0.06 0.06 0.06]

	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
)

define simple_robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	gui_nose 1
	drive "diff"
	color "red"

	size [0.5625 0.5 0.25]

	
	block(
		points 4
		point[0] [0 0.25]
		point[1] [0.125 0.25]
		point[2] [0.125 1.25]
		point[3] [0 1.25]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.125 0.25]
		point[1] [0.75 0.25]
		point[2] [0.75 1]
		point[3] [0.125 0.75]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.75 0.25]
		point[1] [1.5 0.25]
		point[2] [1.5 1.25]
		point[3] [0.75 1.25]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.75 1.25]
		point[1] [1.5 1.25]
		point[2] [1.25 2]
		point[3] [1 2]
		z [0 1]
	)

	block(
		points 4
		point[0] [1 0]
		point[1] [1.25 0]
		point[2] [1.25 0.25]
		point[3] [1 0.25]
		z [0 1]
	)

	block(
		points 4
		point[0] [1.5 0.25]
		point[1] [2.125 0.25]
		point[2] [2.125 0.75]
		point[3] [1.5 1]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.75 1.25]
		point[1] [1.5 1.25]
		point[2] [1.25 2]
		point[3] [1 2]
		z [0 1]
	)

	block(
		points 4
		point[0] [2.125 0.25]
		point[1] [2.25 0.25]
		point[2] [2.25 1.25]
		point[3] [2.125 1.25]
		z [0 1]
	)



	laser(pose [0.23 0.0 -0.15 0.0])
)