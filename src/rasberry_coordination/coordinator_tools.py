#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
import rospy


def logmsgbreak(total=3):
    """ Print a number of lines using logmsg formatting

    :param total: number of lines to print (default: 3)
    :return: None
    """

    for i in range(total):
        logmsg(category="null")

def logmsg(level="info", category="OTHER", id="empty", msg='', throttle=0, speech=False):
    """ Print formatted log messages to console.

    Functionality offered:
    > Clear division of category, id and message
    > Option to remove msg time
    > Option to highlight key details
    > Ability to filter out certain categories

    In order to retain speed and simplicity, properties are preset internally here for:
    :attr reject_categories: list of tags to ignore (useful for managing debugging tools)
    :attr colour_id: list of id's to colour
    :attr colour_categories: list of categories to colour
    :attr valid_categories: list of accepted categories (used to define column widths)
    :attr quick_print: boolean to define if logging should default to use print()
    :attr use_custom_formatting: boolean to define if logging should use basic rospy.logmsg or formatted variant
    :attr disable_ros_time_printout: boolean to define if logging should omit the rospy.Time.now() in logging (slower)
    :attr reset: text colour to return to after logmsg
    :attr info_colour: colour for rospy log info
    :attr warn_colour: colour for rospy log warn
    :attr err_colour: colour for rospy log error
    :attr green_highlight: highlight colour for colour_id
    :attr yellow_highlight: highlight colour for colour_categories

    :param level: level of verbosity [info, warn, error] [fatal and bug/debug not included]
    :param category: type of logging [robot, picker, task, note]
    :param id: agent_id or task_id used for identification
    :param msg: message to display
    :return: None
    """
    # if category == "DTM": speech=True

    quick_print = False
    if quick_print:
        if category == "null": print("\n")
        else: print(category + " | " + str(id) + " | " + msg)
        return

    use_custom_formatting = True
    disable_ros_time_printout = True  # Can cause visual issues on console such as below:
    # [INFO] [1605509085.140152]: OTHER  | var: 1	#output as false
    # [INFO] OTHER  | var: 1 						#ideal output if true
    # [INFO] OTHER  | var: 1152]:					#rostime char after end of ideal output appear (\b cant reach)
    # TODO: include padding at end of msg

    if category.upper() in reject_categories: return

    if use_custom_formatting:

        ros_time = ''
        if disable_ros_time_printout:
            ros_time = '\b' * 21  # TODO: swap out using \u001b[{n}D

        """ Format category portion of message """
        total_pad_space = max([len(_category) + 1 for _category in valid_categories])
        if category.upper() in valid_categories:
            category_padding = total_pad_space - len(category)
            level_padding = (len(level) - 4)
            cat = category.upper() + (" " * (category_padding - level_padding))
        elif category.upper() == "NULL":
            cat = " " * total_pad_space
        else:
            rospy.logerr("category "+category.upper()+" not registered")
            return

        """ Format ID with conditions for when category or id is empty """
        ids = " " * (13 - len(str(id))) + str(id) + ":"
        if id == "empty": ids = " " * 14

        """ Define colour values for printing """ #TODO: optimise this with re.sub(r'\[.*\]','[]',line)
        reset = '\033[00m'
        info_colour = '\033[38;5;231m\033[0m'
        warn_colour = '\033[38;5;136m'
        err_colour = '\033[38;5;1m'
        green_highlight = '\033[01;32m'
        yellow_highlight = '\033[01;33m'

        # identify base colour for message
        if level == "warn":
            base_colour = warn_colour
        elif level == "error":
            base_colour = err_colour
        else:
            base_colour = info_colour

        # define colours for output base
        c1 = reset + base_colour
        c2 = reset + base_colour
        c3 = reset + base_colour
        c4 = reset + base_colour

        # highlight id and/or category based on definitions
        if str(category).upper() in colour_categories:
            c1 = yellow_highlight
        if str(id) in colour_id:
            c3 = colour_id[str(id)]

        basic_msg = msg
        msg = ros_time + "%s%s%s|%s%s %s%s%s" % (c1, cat, c2, c3, ids, c4, msg, reset)
    else:
        if category == "null": return
        msg = category + " | " + str(id) + " | " + msg

    # log in different manners based on the severity level and throttling
    if throttle:
        throttles = {"info": rospy.loginfo_throttle,
                     "warn": rospy.logwarn_throttle,
                     "error": rospy.logerr_throttle}
        throttles[level](throttle, msg)
    else:
        logs = {"info": rospy.loginfo,
                "warn": rospy.logwarn,
                "error": rospy.logerr}
        logs[level](msg)


    if id=="empty": id='';
    if speech: os.system('spd-say "%s, %s" -r 10 -t female2 -w'%(id, basic_msg));



import subprocess
import rasberry_des.config_utils

result = subprocess.check_output('rospack find rasberry_coordination', shell=True)
config_file = result[:-1]+"/src/rasberry_coordination/logging_config/logmsg.yaml"
config_data = rasberry_des.config_utils.get_config_data(config_file)

valid_categories =  [D['name'] for D in config_data['categories'] if not 'reject' in D]
reject_categories = [D['name'] for D in config_data['categories'] if 'reject' in D]
colour_categories =  [D['name'] for D in config_data['categories'] if (not 'reject' in D) and ('colour' in D) and D['colour']]

colour_id = {D['name']:'\033[01;%s'%D['colour'] for D in config_data['ids']}


