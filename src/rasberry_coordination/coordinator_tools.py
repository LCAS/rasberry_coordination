import rospy

def logmsg(level="info", category="OTHER", id="empty", msg=''):

	disable_ros_time_printout = True  # Can cause visual issues on console such as below:
	# [INFO] [1605509085.140152]: OTHER  | var: 1	#output as false
	# [INFO] OTHER  | var: 1 						#ideal output if true
	# [INFO] OTHER  | var: 1152]:					#rostime char after end of ideal output appear (\b cant reach)
	# TODO: include padding at end of msg

	ros_time = ''
	if disable_ros_time_printout:
		ros_time = '\b'*21

	# define id and/or category to highlight
	color_id = ["picker02", "thorvald_001"]
	color_category = ["ROB_PY", "EXEC", "DESPE"]  # TODO move these out of this definition and into some config file
	# (load from paramater server in launch file?)

	# format category portion of message
	valid_categories = ["ROBOT", "PICKER", "TASK", "OTHER", "PSM", "ROB_PY", "EXEC", "FOLLOW"]
	total_padd_space = max([len(_category)+1 for _category in valid_categories])
	if category.upper() in valid_categories:
		category_padding = total_padd_space-len(category)
		level_padding = (len(level)-4)
		cat = category.upper() + (" "*(category_padding+level_padding))

	# format id with conditions for when category or id is empty
	if category == "other":
		id = "coordinator"
	ids = " "*(13-len(str(id)))+str(id)+":"
	if id == "empty":
		ids = " "*14

	# define color values for printing
	reset = '\033[00m'
	info_color = '\033[38;5;231m'
	warn_color = '\033[38;5;136m'
	err_color = '\033[38;5;1m'
	green_highlight = '\033[01;32m'
	yellow_highlight = '\033[01;33m'

	# identify base color for message
	if level == "warn":
		base_color = warn_color
	elif level == "error":
		base_color = err_color
	else:
		base_color = info_color

	# define colors for output base
	c1 = reset+base_color
	c2 = reset+base_color
	c3 = reset+base_color
	c4 = reset+base_color

	# highlight id and/or category based on definitions
	if str(category).upper() in color_category:
		c1 = yellow_highlight
	if str(id) in color_id:
		c3 = green_highlight

	# log in different manners based on the severity level
	color_set = (c1, cat, c2, c3, ids, c4, msg, reset)
	if level == "warn":
		rospy.logwarn(ros_time + "%s%s%s|%s%s %s%s%s" % color_set)
	elif level == "error":
		rospy.logerr(ros_time + "%s%s%s|%s%s %s%s%s" % color_set)
	else:
		rospy.loginfo(ros_time + "%s%s%s|%s%s %s%s%s" % color_set)
