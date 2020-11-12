import rospy

def logmsg(level="info", category="other", id="empty", msg=''):

	# define id and/or category to highlight
	color_id = ["picker01", "picker02", "thorvald_001", "thorvald_002"]
	color_category = ["PICKER"]  # TODO move these out of this definition and into some config file

	# format category portion of message
	types = ["ROBOT", "PICKER", "TASK", "OTHER"]
	padding = [2, 1, 3, 2]
	categories = ["robot", "picker", "task", "other"]
	if not category.lower() in categories:
		print('error with logmsg(), unknown category %s, valid categories are: [%s]' % (category, ', '.join(categories)))
		return
	idx = categories.index(category.lower())
	cat = types[idx]+(" "*(padding[idx]-(len(level)-4)))  # "error" has extra character in tag

	# format id with conditions for when category or id is empty
	if category == "other":
		id = "coordinator"
	ids = " "*(13-len(str(id)))+str(id)+":"
	if id == "empty":
		ids = ""

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
		rospy.logwarn("%s%s %s| %s%s %s%s%s" % color_set)
	elif level == "error":
		rospy.logerr("%s%s %s| %s%s %s%s%s" % color_set)
	else:
		rospy.loginfo("%s%s %s| %s%s %s%s%s" % color_set)
